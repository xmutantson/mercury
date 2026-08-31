// ============================================================================
// KX mid-stream AEAD payload TAMPER harness — the channel-MITM fire proof
// ============================================================================
//
// CLI: mercury.exe --test  ([TEST-KX-DATA-TAMPER] block)
//
// Fact-doc: mercury/fact-documents/data-flow-hybrid-kex.md (live G3 tamper) /
//           mercury/fact-documents/data-flow-aead-nonce.md
//
// THE GAP THIS CLOSES. The encrypted DATA-plane transport activates and delivers
// (pinned-cfg6 8/8, robust-start 8/8), and the KX HANDSHAKE tamper is already
// proven (a tampered reverse ct diverges the key -> confirm-tag mismatch ->
// KEY_ACTIVATE refuse, in test_kx_roundtrip_derive). What was owed is a live
// MID-STREAM PAYLOAD AEAD tamper: a channel man-in-the-middle that flips
// CIPHERTEXT bytes on the POST-ACTIVATION data stream, and proof that the
// receiver REJECTS it — never delivers corrupted plaintext, never silently
// falls back to plaintext.
//
// This harness injects the tamper IN-PROCESS at the wire byte offset (the
// simplest vehicle that exercises the REAL AEAD decrypt path), in two layers:
//
//   PART A — AEAD PRIMITIVE tamper matrix. Two cipher suites are brought to the
//     SAME active session key by a REAL X25519 exchange + derive (the production
//     pairing). A real DATA batch is sealed with the production encrypt(); every
//     wire position a MITM could flip is corrupted and fed to the production
//     decrypt(); each position MUST auth-fail (return <= 0), and a CLEAN copy
//     MUST round-trip (non-vacuity). Positions: forward-ct (CMD->RSP) first byte,
//     mid-payload, last body byte, AEAD tag byte, an 8-byte run; reverse-ct
//     (RSP->CMD) mid-payload; plus the nonce/header binding (correct ct but WRONG
//     wire bsi, and WRONG direction — the reconstructed AEAD nonce diverges).
//
//   PART B — PRODUCTION arq RX-funnel reject (THE fire proof). The batch is
//     driven through the REAL copy_data_to_buffer() delivery funnel — the SAME
//     production path the live responder/commander runs — with the controller's
//     own cipher_suite active. A CLEAN encrypted batch DELIVERS its full payload
//     to the app FIFO (fifo_buffer_rx, the byte oracle) byte-identical; a
//     TAMPERED batch is REJECTED: the production decrypt returns <= 0 -> the
//     funnel prints [CRYPTO] Decrypt FAILED, tears the link (link_status=DROPPED),
//     bumps consecutive_auth_failures, delivers ZERO bytes to the app, and NEVER
//     falls back to plaintext (the auth-fail path goto's PAST every delivery
//     branch — there is no plaintext-fallback edge). Positions cover the forward
//     (CMD->RSP, the core gap) direction and the reverse (RSP->CMD) direction:
//     mid-payload flip, AEAD tag flip, an 8-byte run, and a header/nonce flip.
//
// FAIL-BEFORE / CONTROL: the CLEAN production arm (B0) DELIVERS (occupancy ==
// payload, link up, 0 authfails, bytes byte-identical), so the reject arms are
// meaningful, not a dead path. Each arm re-pairs a fresh session key and
// re-inits the compressor, because the auth-fail branch routes through
// reset_session_state() (which wipes the cipher + deinits the compressor) — so
// every arm is self-contained and order-independent.
//
// In-process synthetic-fire; no RF, no sockets, no snd-aloop cards. Returns 0 on
// PASS, else the failure count. Wired via --test.
// ============================================================================

#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"
#include "crypto/mercury_crypto.h"
#include "compression/mercury_compress.h"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>

// Bring two cipher suites to the SAME active session key via a REAL 2-party
// X25519 exchange + derive from the shared secret + a fixed PSK + fixed callsigns
// on BOTH ends (the production derive inputs). Each suite holds its own ephemeral
// keypair, so a self-DH would NOT match — the exchange is what makes the key
// shared. Mirrors make_paired_suites() in test_aead_nonce.cc.
static bool kx_tamper_pair(cl_cipher_suite& a, cl_cipher_suite& b)
{
	uint8_t pk_a[X25519_KEY_SIZE];
	uint8_t pk_b[X25519_KEY_SIZE];
	if(a.generate_x25519_keypair(pk_a) != 0 ||
	   b.generate_x25519_keypair(pk_b) != 0)
		return false;
	if(a.compute_x25519_shared(pk_b) != 0 ||
	   b.compute_x25519_shared(pk_a) != 0)
		return false;
	const char* psk = "MERCURY-KX-DATA-TAMPER-TEST-PSK";
	a.derive_session_key("CMDTEST", "RSPTEST",
	                     (const uint8_t*)psk, (int)strlen(psk), false);
	b.derive_session_key("CMDTEST", "RSPTEST",
	                     (const uint8_t*)psk, (int)strlen(psk), false);
	a.activate();
	b.activate();
	return true;
}

int cl_arq_controller::test_kx_data_tamper()
{
	int failures = 0;
	auto check = [&](bool cond, const char* name){
		if(!cond){ failures++; printf("[TEST-KX-DATA-TAMPER] FAIL: %s\n", name); }
		else      { printf("[TEST-KX-DATA-TAMPER] ok: %s\n", name); }
		fflush(stdout);
	};

	// ====================================================================== //
	// PART A — AEAD PRIMITIVE tamper matrix (both directions)                 //
	// ====================================================================== //
	{
		cl_cipher_suite tx, rx;
		bool paired = kx_tamper_pair(tx, rx);
		check(paired && tx.is_active() && rx.is_active(),
			"PART A: paired suites active (shared key)");
		if(!paired || !tx.is_active() || !rx.is_active())
			return failures;

		// A realistic mid-stream DATA batch payload.
		const int PT = 200;
		uint8_t pt[PT];
		for(int i=0;i<PT;i++) pt[i] = (uint8_t)(0x11 + i*7);

		// Seal `pt` for a direction at wire bsi 0 (production encrypt()).
		auto seal = [&](uint32_t dir, uint8_t* out, int outcap)->int{
			uint64_t e=0; int l=-1;
			uint64_t idx = cl_cipher_suite::fold_gen_index(
				0, cl_cipher_suite::unwrap_batch_index(0, &e, &l));
			return tx.encrypt(pt, PT, out, outcap, idx, dir, AUTH_TAG_SIZE);
		};
		// Open a (possibly tampered) blob under a chosen wire bsi + direction
		// (production decrypt()). Fresh unwrap state so RX reconstructs the nonce
		// exactly as the live receiver does.
		auto open_at = [&](const uint8_t* blob, int blen, int wire_bsi,
		                   uint32_t dir, uint8_t* out, int outcap)->int{
			uint64_t e=0; int l=-1;
			uint64_t idx = cl_cipher_suite::fold_gen_index(
				0, cl_cipher_suite::unwrap_batch_index(wire_bsi, &e, &l));
			return rx.decrypt(blob, blen, out, outcap, idx, dir, AUTH_TAG_SIZE);
		};

		// ---- forward direction (CMD->RSP, dir=0 — the live DATA direction) ----
		uint8_t ctf[PT + AUTH_TAG_SIZE];
		int nf = seal(DIRECTION_CMD_TO_RSP, ctf, sizeof(ctf));
		bool nf_ok = nf >= 0 && nf <= (int)sizeof(ctf)
			&& nf == PT + AUTH_TAG_SIZE;
		check(nf_ok, "forward: production encrypt sealed batch (len == pt+tag)");
		if(!nf_ok) return failures;

		{	// non-vacuity: a CLEAN forward ct decrypts byte-identical
			uint8_t clean[PT];
			int d = open_at(ctf, nf, 0, DIRECTION_CMD_TO_RSP, clean, sizeof(clean));
			check(d == PT && memcmp(clean, pt, PT)==0,
				"forward CONTROL: clean ct decrypts byte-identical (non-vacuous)");
		}

		// each position: a MITM flip must auth-fail (decrypt <= 0)
		auto flip_reject = [&](int off, int n, const char* label){
			uint8_t t[PT + AUTH_TAG_SIZE];
			memcpy(t, ctf, nf);
			for(int k=0;k<n;k++) t[off+k] ^= 0x5A;
			uint8_t out[PT];
			int d = open_at(t, nf, 0, DIRECTION_CMD_TO_RSP, out, sizeof(out));
			check(d <= 0, label);
		};
		flip_reject(0,        1, "forward-ct FIRST byte flip -> AEAD auth-fail (reject)");
		flip_reject(PT/2,     1, "forward-ct MID-PAYLOAD byte flip -> AEAD auth-fail (reject) [core gap]");
		flip_reject(PT-1,     1, "forward-ct LAST body byte flip -> AEAD auth-fail (reject)");
		flip_reject(PT+3,     1, "forward AEAD TAG byte flip -> auth-fail (reject)");
		flip_reject(PT/3,     8, "forward-ct 8-byte RUN flip -> auth-fail (reject)");

		{	// nonce/header binding: intact ct opened at the WRONG wire bsi
			uint8_t out[PT];
			int d = open_at(ctf, nf, 7 /*wrong bsi*/, DIRECTION_CMD_TO_RSP, out, sizeof(out));
			check(d <= 0, "nonce/header flip: intact ct at WRONG wire bsi -> nonce diverges -> reject");
		}
		{	// direction binding: intact ct opened under the WRONG direction
			uint8_t out[PT];
			int d = open_at(ctf, nf, 0, DIRECTION_RSP_TO_CMD, out, sizeof(out));
			check(d <= 0, "direction flip: intact ct under WRONG direction -> nonce diverges -> reject");
		}

		// ---- reverse direction (RSP->CMD, dir=1) — extend prior coverage -----
		uint8_t ctr[PT + AUTH_TAG_SIZE];
		int nr = seal(DIRECTION_RSP_TO_CMD, ctr, sizeof(ctr));
		bool nr_ok = nr >= 0 && nr <= (int)sizeof(ctr)
			&& nr == PT + AUTH_TAG_SIZE;
		check(nr_ok, "reverse: production encrypt sealed batch (len == pt+tag)");
		if(!nr_ok) return failures;
		{	// non-vacuity
			uint8_t clean[PT];
			int d = open_at(ctr, nr, 0, DIRECTION_RSP_TO_CMD, clean, sizeof(clean));
			check(d == PT && memcmp(clean, pt, PT)==0,
				"reverse CONTROL: clean ct decrypts byte-identical (non-vacuous)");
		}
		{	// reverse-ct mid-payload flip
			uint8_t t[PT + AUTH_TAG_SIZE];
			memcpy(t, ctr, nr);
			t[PT/2] ^= 0x5A;
			uint8_t out[PT];
			int d = open_at(t, nr, 0, DIRECTION_RSP_TO_CMD, out, sizeof(out));
			check(d <= 0, "reverse-ct MID-PAYLOAD byte flip -> AEAD auth-fail (reject)");
		}
	}

	// ====================================================================== //
	// PART B — PRODUCTION arq RX-funnel reject (THE fire proof)               //
	// ====================================================================== //
	// Fixture: a compression-viable OFDM config; the controller's OWN cipher_suite
	// is the RX peer, a standalone tx cipher is the sender; a real compressed batch
	// (compress -> encrypt, the production order) is driven through the REAL
	// copy_data_to_buffer() funnel.
	this->nMessages          = 120;
	this->max_data_length    = 170;
	this->max_message_length = 200;
	this->max_header_length  = 6;
	if(init_messages_buffers() != SUCCESSFUL){
		printf("[TEST-KX-DATA-TAMPER] ERROR: init_messages_buffers() failed\n");
		fflush(stdout);
		printf("[TEST-KX-DATA-TAMPER] FAIL (failures=%d)\n", failures+1);
		fflush(stdout);
		return failures + 1;
	}
	this->sack_v2_enabled   = true;
	this->sack_enabled      = true;
	this->header_carries_d5 = true;

	// The AEAD auth-fail production path routes through reset_session_state(), which
	// writes three SCALAR fields on telecom_system (narrowband_enabled,
	// current_configuration, last_coarse_freq_offset — all default-constructed). A
	// bare synthetic-fire controller has no telecom_system, so give it one; no PHY
	// load_configuration() is needed for these scalar writes. Nulled before return so
	// the controller destructor never touches this stack object once it goes away.
	cl_telecom_system kx_ts;
	this->telecom_system = &kx_ts;

	const int APP = 100;
	char app[APP];
	for(int i=0;i<APP;i++) app[i] = "MERCURY EMERGENCY "[i % 18];

	// One self-contained production-path arm.
	//   role_for_rx : COMMANDER (RX receives RSP->CMD = reverse) or RESPONDER
	//                 (RX receives CMD->RSP = forward, the core-gap direction)
	//   kind        : 0 CLEAN · 1 mid-payload · 2 tag byte · 3 8-byte run ·
	//                 4 header/nonce (wrong bsi, ct intact) · 5 first byte
	auto run_arm = [&](int role_for_rx, int kind,
	                   int* out_occ, bool* out_dropped, int* out_authfail,
	                   bool* out_match)->void
	{
		// --- fresh fixture (a fail arm's reset_session_state wipes cipher+compressor) ---
		this->original_role          = role_for_rx;
		this->role                   = role_for_rx;
		this->narrowband_enabled     = NO;    // WB (config 0); a prior arm's auth-fail
		                                      // reset_session_state() forces NB restore
		this->current_configuration  = 0;     // OFDM non-robust -> viable, tag 16
		this->data_configuration     = 0;
		this->init_configuration     = 0;
		this->rx_copy_window         = -1;
		this->compressor.deinit();
		this->compressor.init();
		this->compression_enabled    = true;
		this->encryption_enabled     = true;

		// pair the controller's own cipher_suite (RX) with a fresh sender tx
		this->cipher_suite.wipe();
		cl_cipher_suite tx;
		bool paired = kx_tamper_pair(tx, this->cipher_suite);
		if(!paired){
			check(false, "production path key exchange setup");
			if(out_occ) *out_occ = -1;
			if(out_dropped) *out_dropped = false;
			if(out_authfail) *out_authfail = 0;
			if(out_match) *out_match = false;
			return;
		}

		uint32_t dir = (role_for_rx == COMMANDER)
			? DIRECTION_RSP_TO_CMD : DIRECTION_CMD_TO_RSP;

		// build a REAL compressed batch, then seal it (production compress->encrypt)
		char comp[512];
		int comp_len = this->compressor.compress_block(app, APP, comp, sizeof(comp));

		uint8_t wire[1024];
		uint64_t e=0; int l=-1;
		uint64_t bidx = cl_cipher_suite::fold_gen_index(
			0, cl_cipher_suite::unwrap_batch_index(0, &e, &l));
		int wlen = tx.encrypt((const uint8_t*)comp, comp_len,
			wire, sizeof(wire), bidx, dir, AUTH_TAG_SIZE);

		bool wire_ok = comp_len > 0
			&& comp_len <= (int)sizeof(wire) - AUTH_TAG_SIZE
			&& wlen >= 0 && wlen <= (int)sizeof(wire)
			&& wlen <= N_MAX / 8
			&& wlen == comp_len + AUTH_TAG_SIZE;
		check(wire_ok, "production encrypt returned a bounded wire length");
		if(!wire_ok){
			if(out_occ) *out_occ = -1;
			if(out_dropped) *out_dropped = false;
			if(out_authfail) *out_authfail = 0;
			if(out_match) *out_match = false;
			return;
		}

		// apply the wire tamper for this position
		bool wrong_bsi = false;
		if(comp_len > 1 && wlen > 0){
			switch(kind){
				case 0: break;                                   // clean
				case 1: wire[comp_len/2]        ^= 0x5A; break;  // body mid
				case 2: wire[comp_len + AUTH_TAG_SIZE/2] ^= 0x5A; break; // tag byte
				case 3: for(int k=0;k<8;k++) wire[comp_len/2 + k] ^= 0x5A; break; // 8-byte run
				case 4: wrong_bsi = true; break;                 // header/nonce (ct intact)
				case 5: wire[0]                 ^= 0x5A; break;  // first byte
			}
		}

		// --- reset RX delivery + nonce state so the funnel-top guards are inert ---
		rx_stream_emitted_bsi_hw     = -1;
		rx_stream_delivered          = 0;
		rsp_cross_session_seam_armed = false;
		for(int s=0;s<256;s++) rx_stream_stamp[s].valid = false;
		rx_nonce_epoch    = 0;
		rx_nonce_last_bsi = -1;
		rx_nonce_gen      = 0;
		// header/nonce flip: the RX reconstructs the nonce from a DIFFERENT wire bsi
		decrypt_delivered_bsi     = wrong_bsi ? 1 : 0;
		consecutive_auth_failures = 0;
		this->link_status         = CONNECTED;

		// --- stage the (possibly tampered) wire ct as a single ACKED DATA frame ---
		this->data_batch_size = 1;
		for(int i=0;i<this->nMessages;i++){ messages_rx[i].status=FREE; messages_rx[i].length=0; }
		memcpy(messages_rx[0].data, wire, wlen);
		messages_rx[0].length = wlen;
		messages_rx[0].status = ACKED;

		fifo_buffer_rx.set_size(8192);
		fifo_buffer_rx.flush();

		// --- drive the REAL production delivery funnel (decrypt lives here) ---
		copy_data_to_buffer();

		int occ = fifo_buffer_rx.get_size() - fifo_buffer_rx.get_free_size();
		if(out_occ)      *out_occ      = occ;
		if(out_dropped)  *out_dropped  = (this->link_status == DROPPED);
		if(out_authfail) *out_authfail = consecutive_auth_failures;
		if(out_match){
			*out_match = false;
			if(occ == APP){
				char got[APP];
				int gn = fifo_buffer_rx.pop(got, APP);
				*out_match = (gn == APP && memcmp(got, app, APP)==0);
			}
		}
	};

	int occ; bool dropped; int af; bool match;

	// ---- B0 CONTROL (RSP->CMD): clean encrypted batch DELIVERS full payload ----
	run_arm(COMMANDER, 0, &occ, &dropped, &af, &match);
	check(occ == APP && !dropped && af == 0 && match,
		"PROD CONTROL (RSP->CMD): clean batch DELIVERS full payload byte-identical, link up, 0 authfail");

	// ---- reverse-ct (RSP->CMD) reject positions ----
	run_arm(COMMANDER, 1, &occ, &dropped, &af, nullptr);
	check(occ == 0 && dropped,
		"PROD reverse-ct MID-PAYLOAD flip: 0 bytes to app, link DROPPED, authfail fired");
	run_arm(COMMANDER, 2, &occ, &dropped, &af, nullptr);
	check(occ == 0 && dropped,
		"PROD reverse AEAD TAG flip: 0 bytes to app, link DROPPED, authfail fired");
	run_arm(COMMANDER, 3, &occ, &dropped, &af, nullptr);
	check(occ == 0 && dropped,
		"PROD reverse-ct 8-byte RUN flip: 0 bytes to app, link DROPPED, authfail fired");
	run_arm(COMMANDER, 4, &occ, &dropped, &af, nullptr);
	check(occ == 0 && dropped,
		"PROD header/nonce flip (wrong wire bsi): 0 bytes to app, link DROPPED, authfail fired");

	// ---- forward-ct (CMD->RSP) — the CORE GAP direction — reject on the prod path ----
	run_arm(RESPONDER, 1, &occ, &dropped, &af, nullptr);
	check(occ == 0 && dropped,
		"PROD forward-ct (CMD->RSP) MID-PAYLOAD flip [core gap]: 0 bytes to app, link DROPPED, authfail fired");
	run_arm(RESPONDER, 2, &occ, &dropped, &af, nullptr);
	check(occ == 0 && dropped,
		"PROD forward AEAD TAG flip (CMD->RSP): 0 bytes to app, link DROPPED, authfail fired");

	// Detach the stack telecom_system before it (and this frame) unwind, so the
	// controller's destructor in the caller never dereferences a dangling pointer.
	this->telecom_system = NULL;

	printf("[TEST-KX-DATA-TAMPER] %s (failures=%d)\n",
		failures ? "FAIL" : "ALL PASS", failures);
	fflush(stdout);
	return failures;
}
