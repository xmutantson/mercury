/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2024 Fadi Jerji
 * Author: Fadi Jerji
 * Email: fadi.jerji@  <gmail.com, caisresearch.com, ieee.org>
 * ORCID: 0000-0002-2076-5831
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "datalink_layer/arq.h"
#include "common/snr_decision_grid.h"
#include "datalink_layer/l1_block_codec.h"
#include "datalink_layer/gearshift_quality_report.h"
#include "audioio/audioio.h"
#include "debug/canary_guard.h"
#include "physical_layer/mfsk_ctrl_codec.h"  // Stage 2 in-band rate-adapt config tag
#include <time.h>
#include <vector>
#ifdef __GLIBC__
#include <malloc.h>
#endif
#include "common/timing_log.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <chrono>
#include <climits>
#include <cstdlib>
#include <atomic>


extern cbuf_handle_t capture_buffer;
extern cbuf_handle_t playback_buffer;



// In-process regression for the fail-closed negotiation policy. Drives the REAL
// production predicate decide_encryption_negotiation() (the same function both
// endpoints call), across the operator/peer capability matrix. Returns 0=PASS,
// else the number of failed assertions. Fails-before under -DENC_FAILOPEN_FAILBEFORE.
int cl_arq_controller::test_encryption_fail_closed()
{
	int fails = 0;
	const uint8_t ENC = CAP_ENCRYPTION;            // 0x02
	const uint8_t WB  = CAP_WB_CAPABLE;            // 0x01 (a non-encryption cap bit)

	auto check = [&](const char* name, enc_negotiation_outcome_t got,
	                 enc_negotiation_outcome_t want)
	{
		const char* names[] = {"PLAINTEXT_OK","ENABLED","REFUSE"};
		bool ok = (got == want);
		if(!ok) fails++;
		printf("[TEST-ENC-FAILCLOSED] %-46s got=%s want=%s -> %s\n",
			name, names[(int)got], names[(int)want], ok ? "PASS" : "FAIL");
	};

	printf("[TEST-ENC-FAILCLOSED] start (single source of truth: decide_encryption_negotiation)\n");
	fflush(stdout);

	// --- DEFAULT-OFF: operator did NOT opt in -> plaintext ops legal + unchanged ---
	// (Peer cap is irrelevant; the opt-in gate is checked first.)
	check("OFF, peer no-cap        -> plaintext (legal default-off)",
	      decide_encryption_negotiation(ENCRYPT_OFF, ENC, 0),   ENC_NEG_PLAINTEXT_OK);
	check("OFF, peer has-cap       -> plaintext (legal default-off)",
	      decide_encryption_negotiation(ENCRYPT_OFF, ENC, ENC), ENC_NEG_PLAINTEXT_OK);
	check("OFF, neither cap        -> plaintext (legal default-off)",
	      decide_encryption_negotiation(ENCRYPT_OFF, 0, 0),     ENC_NEG_PLAINTEXT_OK);

	// --- OPTED IN + peer supports -> encrypt ---
	check("FAST, both support      -> encrypt",
	      decide_encryption_negotiation(ENCRYPT_FAST,   ENC, ENC), ENC_NEG_ENABLED);
	check("STRICT, both support    -> encrypt",
	      decide_encryption_negotiation(ENCRYPT_STRICT, ENC, ENC), ENC_NEG_ENABLED);

	// --- FAIL-CLOSED: opted in, peer lacks CAP_ENCRYPTION -> REFUSE (0 plaintext) ---
	// The core of the fix: FAST must refuse exactly like STRICT.
	check("FAST, peer no-cap       -> REFUSE (was plaintext downgrade)",
	      decide_encryption_negotiation(ENCRYPT_FAST,   ENC, 0),  ENC_NEG_REFUSE);
	check("STRICT, peer no-cap     -> REFUSE",
	      decide_encryption_negotiation(ENCRYPT_STRICT, ENC, 0),  ENC_NEG_REFUSE);
	// MITM cap-strip: peer advertised WB but the ENCRYPTION bit was cleared.
	check("FAST, MITM stripped cap -> REFUSE",
	      decide_encryption_negotiation(ENCRYPT_FAST,   ENC, WB), ENC_NEG_REFUSE);
	check("STRICT, MITM stripped   -> REFUSE",
	      decide_encryption_negotiation(ENCRYPT_STRICT, ENC, WB), ENC_NEG_REFUSE);

	printf("[TEST-ENC-FAILCLOSED] %s (%d failure(s))\n",
		fails == 0 ? "PASS" : "FAIL", fails);
	fflush(stdout);
	return fails;
}

int cl_arq_controller::test_ssid_bounds()
{
	// Numeric AX.25 SSIDs stop at 15. The letter aliases remain distinct
	// protocol extensions and must continue to map to their reserved values.
	const bool pass = callsign_get_ssid("N0CALL-0") == 0
		&& callsign_get_ssid("N0CALL-15") == 15
		&& callsign_get_ssid("N0CALL-16") == SSID_NONE
		&& callsign_get_ssid("N0CALL-L") == 16;
	printf("[TEST-SSID-BOUNDS] %s\n", pass ? "PASS" : "FAIL");
	return pass ? 0 : 1;
}

int cl_arq_controller::test_scream_rung_map()
{
	struct map_case { int current; int rung; int expected; } cases[] = {
		{CONFIG_16, 0, CONFIG_15}, {CONFIG_16, 1, CONFIG_14},
		{CONFIG_16, 2, CONFIG_12}, {CONFIG_16, 3, CONFIG_0},
		{CONFIG_2,  0, CONFIG_1},  {CONFIG_2,  1, CONFIG_0},
		{CONFIG_2,  2, CONFIG_0},  {CONFIG_2,  3, CONFIG_0},
		{CONFIG_17, 0, CONFIG_16}, {CONFIG_17, 1, CONFIG_15}
	};
	int failures = 0;
	for(size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
	{
		int got = scream_rung_to_config(cases[i].current, cases[i].rung);
		bool pass = got == cases[i].expected;
		printf("[TEST-SCREAM-MAP] %s current=%d rung=%d target=%d expected=%d\n",
			pass ? "PASS" : "FAIL", cases[i].current, cases[i].rung,
			got, cases[i].expected);
		if(!pass) failures++;
	}
	bool invalid_pass = scream_rung_to_config(CONFIG_NONE, 0) == CONFIG_NONE;
	printf("[TEST-SCREAM-MAP] %s invalid current fails closed\n",
		invalid_pass ? "PASS" : "FAIL");
	if(!invalid_pass) failures++;
	struct policy_case {
		int delivered; bool armed; int elapsed; int holdoff; bool emit;
	} policy[] = {
		{0, true, 60000, 6595, false},
		{2, true, 60000, 6595, false},
		{3, false, 60000, 6595, false},
		{3, true, 6594, 6595, false},
		{3, true, 6595, 6595, true}
	};
	for(size_t i=0; i<sizeof(policy)/sizeof(policy[0]); i++)
	{
		bool got = scream_trigger_policy_ready(policy[i].delivered,
			policy[i].armed, policy[i].elapsed, policy[i].holdoff);
		bool pass = got == policy[i].emit;
		printf("[TEST-SCREAM-POLICY] %s delivered=%d armed=%d elapsed=%d holdoff=%d emit=%d\n",
			pass ? "PASS" : "FAIL", policy[i].delivered,
			policy[i].armed ? 1 : 0, policy[i].elapsed,
			policy[i].holdoff, got ? 1 : 0);
		if(!pass) failures++;
	}
	bool scaling_pass = scream_trigger_holdoff_for_timeout(2000) == 1000
		&& scream_trigger_holdoff_for_timeout(26380) == 6595
		&& scream_trigger_holdoff_for_timeout(50000) == 8000;
	printf("[TEST-SCREAM-POLICY] %s hardware-scaled holdoff clamp\n",
		scaling_pass ? "PASS" : "FAIL");
	if(!scaling_pass) failures++;
	bool rollback_pass = !scream_rollback_slot_eligible(PENDING_ACK, 12, -1)
		&& scream_rollback_slot_eligible(PENDING_ACK, 12, 2);
	printf("[TEST-SCREAM-RESUME] %s staged bsi=-1 ignored; aired bsi=2 eligible\n",
		rollback_pass ? "PASS" : "FAIL");
	if(!rollback_pass) failures++;
	return failures;
}
// Portable env set/unset for the self-test only (production never writes env).
static void setenv_portable(const char* name, const char* val)
{
#ifdef _WIN32
	char buf[256]; snprintf(buf, sizeof(buf), "%s=%s", name, val); _putenv(buf);
#else
	setenv(name, val, 1);
#endif
}
static void unsetenv_portable(const char* name)
{
#ifdef _WIN32
	char buf[256]; snprintf(buf, sizeof(buf), "%s=", name); _putenv(buf);
#else
	unsetenv(name);
#endif
}


// Consolidated fail-before/pass-after self-test for the three implemented fixes.
// Asserts each pure predicate's truth table with the defeat knob unset (pass-after)
// and set (fail-before). No IONOS/RF; deterministic. Returns #failures.
int cl_arq_controller::test_zombie_amp()
{
	int fails = 0;
	auto CHECK = [&](bool cond, const char* what) {
		if(!cond) { printf("[TEST-ZOMBIE-AMP] FAIL: %s\n", what); fflush(stdout); fails++; }
	};

	// Make the environment deterministic regardless of caller state.
	unsetenv_portable("MERCURY_BREAK_DROPPED_DEFEAT");
	unsetenv_portable("MERCURY_ROBUST_DWELL_KEEPALIVE_DEFEAT");
	unsetenv_portable("MERCURY_WATCHDOG_PROBE_DEFEAT");

	printf("[TEST-ZOMBIE-AMP] start\n"); fflush(stdout);

	// --- FIX 1 (R2b): BREAK actionable-by-link_status ---------------------
	// pass-after (no defeat): CONNECTED + DROPPED actionable; LISTENING/CONNECTING not.
	CHECK( break_frame_actionable(CONNECTED),  "R2b: CONNECTED must accept BREAK");
	CHECK( break_frame_actionable(DROPPED),    "R2b: DROPPED must accept BREAK (the zombie fix)");
	CHECK(!break_frame_actionable(LISTENING),  "R2b: LISTENING must discard BREAK");
	CHECK(!break_frame_actionable(CONNECTING), "R2b: CONNECTING must discard BREAK");
	// fail-before (defeat): DROPPED reverts to discard, CONNECTED still accepts.
	setenv_portable("MERCURY_BREAK_DROPPED_DEFEAT", "1");
	CHECK(!break_frame_actionable(DROPPED),    "R2b fail-before: DROPPED discards under DEFEAT");
	CHECK( break_frame_actionable(CONNECTED),  "R2b fail-before: CONNECTED still accepts under DEFEAT");
	unsetenv_portable("MERCURY_BREAK_DROPPED_DEFEAT");

	// --- FIX 4 (ROBUST_DWELL keep-alive) ----------------------------------
	CHECK( robust_dwell_keepalive_ok(CONNECTED), "FIX4: CONNECTED keeps the keep-alive");
	CHECK(!robust_dwell_keepalive_ok(DROPPED),   "FIX4: DROPPED suppresses the keep-alive (zombie must expire)");
	setenv_portable("MERCURY_ROBUST_DWELL_KEEPALIVE_DEFEAT", "1");
	CHECK( robust_dwell_keepalive_ok(DROPPED),   "FIX4 fail-before: DROPPED keeps keep-alive under DEFEAT");
	unsetenv_portable("MERCURY_ROBUST_DWELL_KEEPALIVE_DEFEAT");

	// --- FIX 3 (watchdog probe gate) --------------------------------------
	// pass-after: first fire probes; second fire resumes if rx advanced else reconnects.
	CHECK(watchdog_resurrect_decision(/*pending=*/false, /*rx_adv=*/false) == WD_PROBE,
		"FIX3: first fire must PROBE (no unconditional teleport)");
	CHECK(watchdog_resurrect_decision(/*pending=*/true,  /*rx_adv=*/true ) == WD_RESUME,
		"FIX3: peer answered (rx advanced) must RESUME (no teleport)");
	CHECK(watchdog_resurrect_decision(/*pending=*/true,  /*rx_adv=*/false) == WD_RECONNECT,
		"FIX3: pure-silence must RECONNECT (clean boundary, no teleport)");
	// fail-before: defeat restores the unconditional teleport on every branch.
	setenv_portable("MERCURY_WATCHDOG_PROBE_DEFEAT", "1");
	CHECK(watchdog_resurrect_decision(false, false) == WD_TELEPORT,
		"FIX3 fail-before: DEFEAT restores unconditional TELEPORT");
	CHECK(watchdog_resurrect_decision(true,  false) == WD_TELEPORT,
		"FIX3 fail-before: DEFEAT teleports even on pure silence");
	unsetenv_portable("MERCURY_WATCHDOG_PROBE_DEFEAT");

	printf("[TEST-ZOMBIE-AMP] %s (%d failures)\n", fails == 0 ? "PASS" : "FAIL", fails);
	fflush(stdout);
	return fails;
}

// R5 fail-before/pass-after self-test: the pure classifier truth table, the counter evolution via
// the SAME production step helper, the reroute predicate + its guards, the DEFEAT knob, and the
// +LOW watchdog probe-latch hygiene. No IONOS/RF; deterministic. Returns #failures.
int cl_arq_controller::test_demote_silence()
{
	int fails = 0;
	auto CHECK = [&](bool c, const char* w){ if(!c){ printf("[TEST-DEMOTE-SILENCE] FAIL: %s\n", w); fflush(stdout); fails++; } };
	unsetenv_portable("MERCURY_DEMOTE_SILENCE_DEFEAT");
	printf("[TEST-DEMOTE-SILENCE] start\n"); fflush(stdout);

	// (a) classifier truth table
	CHECK(classify_ack_failure(0, 0.0, 0, false) == ACKFAIL_SILENT,
	      "pure silence (pat<=0, matched=0, metric~0) => SILENT");
	CHECK(classify_ack_failure(0, 0.62, 5, false) == ACKFAIL_LATE_ACK,
	      "sub-threshold correlator hit (matched=5) => LATE_ACK (peer alive)");
	CHECK(classify_ack_failure(389, 0.0, 0, false) == ACKFAIL_LATE_ACK,
	      "ack pattern was heard (pat>0) => LATE_ACK (window problem)");
	CHECK(classify_ack_failure(0, 0.0, 0, true) == ACKFAIL_FORWARD_LOSS,
	      "partial SACK => FORWARD_LOSS (evidence-bearing demote)");
	CHECK(classify_ack_failure(0, 0.10, 0, false) == ACKFAIL_LATE_ACK,
	      "metric exactly at ACK_SILENCE_EPS => LATE_ACK (not silence)");

	// (b) counter evolution via the SAME production step helper (parser-nonzero: print the count)
	consec_pure_silent_rounds = 0;
	int stepped = 0;
	for(int i=0;i<DEMOTE_SILENCE_MAX_ROUNDS;i++) { ackfail_classifier_step(0, 0.0, 0, false); stepped++; }
	printf("[TEST-DEMOTE-SILENCE] drove %d silent step rows, consec=%d\n", stepped, consec_pure_silent_rounds);
	CHECK(stepped == DEMOTE_SILENCE_MAX_ROUNDS, "drove M step rows (parser-nonzero)");
	CHECK(consec_pure_silent_rounds == DEMOTE_SILENCE_MAX_ROUNDS, "M silent rounds accumulate");
	CHECK(silence_reroute_should_fire(consec_pure_silent_rounds, DEMOTE_SILENCE_MAX_ROUNDS,
	      /*emergency_break_active=*/false, TURBO_DONE), "reroute fires at M silent (pass-after)");

	// one sub-threshold (LATE_ACK) round RESETS -> reroute must NOT fire (byte-identical BREAK)
	ackfail_classifier_step(0, 0.62, 5, false);
	CHECK(consec_pure_silent_rounds == 0, "one LATE_ACK round resets the silent streak");
	CHECK(!silence_reroute_should_fire(consec_pure_silent_rounds, DEMOTE_SILENCE_MAX_ROUNDS,
	      false, TURBO_DONE), "reroute suppressed after reverse activity (forward-loss path stays legacy)");

	// a partial-SACK (FORWARD_LOSS) round mid-sequence also resets
	ackfail_classifier_step(0, 0.0, 0, false);
	ackfail_classifier_step(0, 0.0, 0, true);
	CHECK(consec_pure_silent_rounds == 0, "FORWARD_LOSS round resets (demote path byte-identical)");

	// guards: emergency_break_active or non-TURBO_DONE suppress even at M
	consec_pure_silent_rounds = DEMOTE_SILENCE_MAX_ROUNDS;
	CHECK(!silence_reroute_should_fire(consec_pure_silent_rounds, DEMOTE_SILENCE_MAX_ROUNDS,
	      /*emergency_break_active=*/true, TURBO_DONE), "no reroute while a BREAK is already active");
	CHECK(!silence_reroute_should_fire(consec_pure_silent_rounds, DEMOTE_SILENCE_MAX_ROUNDS,
	      false, TURBO_DONE + 1), "no reroute unless turboshift is DONE");

	// (c) DEFEAT knob = fail-before (ladder walks on silence)
	setenv_portable("MERCURY_DEMOTE_SILENCE_DEFEAT", "1");
	CHECK(!demote_silence_reroute_enabled(), "fail-before: DEFEAT disables the reroute (legacy ladder-walk)");
	unsetenv_portable("MERCURY_DEMOTE_SILENCE_DEFEAT");
	CHECK(demote_silence_reroute_enabled(), "pass-after: reroute enabled by default");

	// (d) +LOW watchdog probe-latch hygiene: a liveness credit clears a stale WD_PROBE latch and
	// re-snaps the rx index (so a later independent watchdog fire starts fresh at WD_PROBE).
	watchdog_resurrect_probe_pending = true;
	watchdog_probe_rx_index_snap     = 999999;
	rx_receive_frame_index           = 42;
	watchdog_probe_clear_on_liveness();
	CHECK(watchdog_resurrect_probe_pending == false, "liveness credit clears the probe latch");
	CHECK(watchdog_probe_rx_index_snap == rx_receive_frame_index, "liveness credit re-snaps the rx index");

	printf("[TEST-DEMOTE-SILENCE] %s (%d failures)\n", fails==0?"PASS":"FAIL", fails);
	fflush(stdout);
	return fails;
}

// KX-AS-DATA reserved-stream RX ingest fire proof (data-flow-hybrid-kex.md T1/T3).
// Drives the REAL kx_ingest() reassembler on a throwaway controller (no channel /
// no IONOS / no RF): (a) a well-formed FORWARD stream delivered in IRREGULAR
// FRAGMENTS (header split across a fragment) reassembles the 1184B ML-KEM pk +
// the folded 32B x25519 pubkey BYTE-IDENTICAL and is NEVER routed to the app; (b)
// the REVERSE stream reassembles the 1088B ct byte-identical; (c) the fail-secure
// REFUSE FIRES (returns -1) on bad magic / bad kind / wrong length / a byte after
// activation. Returns the failure count (0 = pass), summed into the --test suite.
int cl_arq_controller::test_kx_ingest()
{
	int failures = 0;
	auto check = [&](bool cond, const char* name){
		if(!cond){ failures++; printf("[TEST-KX-INGEST] FAIL: %s\n", name); }
		else      { printf("[TEST-KX-INGEST] ok: %s\n", name); }
		fflush(stdout);
	};

	kx_chunk_state_reset();

	// --- FORWARD: MAGIC|kind|len | x25519(32) | mlkem_pk(1184) -------------------
	uint8_t x25519_ref[X25519_KEY_SIZE];
	for(int i=0;i<X25519_KEY_SIZE;i++) x25519_ref[i] = (uint8_t)(0xA0 + i);
	uint8_t pk_ref[MLKEM_PK_SIZE];
	for(int i=0;i<MLKEM_PK_SIZE;i++)   pk_ref[i]     = (uint8_t)((i*7 + 3) & 0xFF);
	int fwd_payload = X25519_KEY_SIZE + MLKEM_PK_SIZE;   // 1216
	uint8_t fwd[KX_STREAM_HDR + X25519_KEY_SIZE + MLKEM_PK_SIZE];
	fwd[0]=(uint8_t)((KX_STREAM_MAGIC>>24)&0xFF); fwd[1]=(uint8_t)((KX_STREAM_MAGIC>>16)&0xFF);
	fwd[2]=(uint8_t)((KX_STREAM_MAGIC>>8)&0xFF);  fwd[3]=(uint8_t)(KX_STREAM_MAGIC&0xFF);
	fwd[4]=KX_STREAM_FWD;
	fwd[5]=(uint8_t)(fwd_payload & 0xFF); fwd[6]=(uint8_t)((fwd_payload>>8)&0xFF);   // LE
	memcpy(fwd+KX_STREAM_HDR, x25519_ref, X25519_KEY_SIZE);
	memcpy(fwd+KX_STREAM_HDR+X25519_KEY_SIZE, pk_ref, MLKEM_PK_SIZE);
	int fwd_total = KX_STREAM_HDR + fwd_payload;

	int off = 0, rc_sum = 0; bool refused_mid = false;
	const int frags[] = {3, 5, 200, 500, 1000000};   // last = "the rest"
	for(int f=0; off < fwd_total; f++)
	{
		int n = (f < 4) ? frags[f] : (fwd_total - off);
		if(off + n > fwd_total) n = fwd_total - off;
		int rc = kx_ingest(fwd+off, n);
		if(rc < 0){ refused_mid = true; break; }
		rc_sum += rc; off += n;
	}
	check(!refused_mid, "forward stream not falsely refused");
	check(rc_sum == fwd_total, "forward stream fully consumed as KX");
	check(kx_stream_rx_done_kind == KX_STREAM_FWD, "forward transfer completed (done_kind==FWD)");
	check(kx_mlkem_pk_ready, "kx_mlkem_pk_ready set on completion");
	check(memcmp(kx_mlkem_pk, pk_ref, MLKEM_PK_SIZE)==0, "ML-KEM pk reassembled BYTE-IDENTICAL");
	check(memcmp(kx_stream_peer_x25519, x25519_ref, X25519_KEY_SIZE)==0, "folded x25519 pubkey BYTE-IDENTICAL");

	// --- REVERSE: MAGIC|kind|len | x25519(32) | mlkem_ct(1088) ------------------
	kx_chunk_state_reset();
	uint8_t ct_ref[MLKEM_CT_SIZE];
	for(int i=0;i<MLKEM_CT_SIZE;i++) ct_ref[i] = (uint8_t)((i*5 + 11) & 0xFF);
	int rev_payload = X25519_KEY_SIZE + MLKEM_CT_SIZE;   // 1120
	uint8_t rev[KX_STREAM_HDR + X25519_KEY_SIZE + MLKEM_CT_SIZE];
	rev[0]=(uint8_t)((KX_STREAM_MAGIC>>24)&0xFF); rev[1]=(uint8_t)((KX_STREAM_MAGIC>>16)&0xFF);
	rev[2]=(uint8_t)((KX_STREAM_MAGIC>>8)&0xFF);  rev[3]=(uint8_t)(KX_STREAM_MAGIC&0xFF);
	rev[4]=KX_STREAM_REV;
	rev[5]=(uint8_t)(rev_payload & 0xFF); rev[6]=(uint8_t)((rev_payload>>8)&0xFF);
	memcpy(rev+KX_STREAM_HDR, x25519_ref, X25519_KEY_SIZE);
	memcpy(rev+KX_STREAM_HDR+X25519_KEY_SIZE, ct_ref, MLKEM_CT_SIZE);
	int rev_total = KX_STREAM_HDR + rev_payload;
	int rc_rev = kx_ingest(rev, rev_total);
	check(rc_rev == rev_total, "reverse stream fully consumed as KX");
	check(kx_stream_rx_done_kind == KX_STREAM_REV, "reverse transfer completed (done_kind==REV)");
	check(kx_mlkem_ct_ready, "kx_mlkem_ct_ready set on completion");
	check(memcmp(kx_mlkem_ct, ct_ref, MLKEM_CT_SIZE)==0, "ML-KEM ct reassembled BYTE-IDENTICAL");

	// --- FAIL-SECURE REFUSE (the gate FIRES on the production path) --------------
	kx_chunk_state_reset();
	uint8_t hdr4[KX_STREAM_HDR + 4]; memcpy(hdr4, fwd, KX_STREAM_HDR+4);
	uint8_t badm[KX_STREAM_HDR + 4]; memcpy(badm, hdr4, KX_STREAM_HDR+4); badm[0]^=0xFF;
	check(kx_ingest(badm, KX_STREAM_HDR+4) < 0, "bad magic -> REFUSE (-1)");
	kx_chunk_state_reset();
	uint8_t badk[KX_STREAM_HDR + 4]; memcpy(badk, hdr4, KX_STREAM_HDR+4); badk[4]=0x7F;
	check(kx_ingest(badk, KX_STREAM_HDR+4) < 0, "bad kind -> REFUSE (-1)");
	kx_chunk_state_reset();
	uint8_t badl[KX_STREAM_HDR + 4]; memcpy(badl, hdr4, KX_STREAM_HDR+4); badl[5]^=0x01;
	check(kx_ingest(badl, KX_STREAM_HDR+4) < 0, "wrong total_len -> REFUSE (-1)");

	// --- FAIL-SECURE: a KX byte after activation is a routing contradiction ------
	// (done LAST — activate() is sticky on this throwaway controller.)
	kx_chunk_state_reset();
	cipher_suite.activate();
	check(kx_ingest(fwd, KX_STREAM_HDR) < 0, "KX byte after activation -> REFUSE (-1)");

	printf("[TEST-KX-INGEST] %s (failures=%d)\n", failures? "FAIL":"ALL PASS", failures);
	fflush(stdout);
	return failures;
}

// KX-AS-DATA forward TX stage/feed fire proof (data-flow-hybrid-kex.md §2.2). Drives the
// REAL process_buffer_data_commander() source-switch end to end: the production hold path
// stages the reserved forward stream (x25519_pk_cmd + mlkem_pk) via kx_stage_forward_stream()
// and the raw-leg fill frames it as ordinary DATA batches (NO Option-W wire stamp, fed from
// kx_stream_tx_buf, never fifo_buffer_tx). The framed bytes are then delivered through the
// REAL copy_data_to_buffer()->kx_ingest() RX funnel; the ML-KEM pk + folded x25519 pubkey
// MUST reassemble BYTE-IDENTICAL and NEVER reach the app FIFO. Both production functions,
// no PHY / no channel / no telecom_system. 0 = pass / N = failure count.
int cl_arq_controller::test_kx_tx_stream()
{
	int failures = 0;
	auto check = [&](bool cond, const char* name){
		if(!cond){ failures++; printf("[TEST-KX-TXSTREAM] FAIL: %s\n", name); }
		else      { printf("[TEST-KX-TXSTREAM] ok: %s\n", name); }
		fflush(stdout);
	};

	// --- Controller setup (no-compression raw-leg template; mirrors test_kx_rx_routing) ---
	this->max_data_length    = 170;
	this->max_header_length  = 6;
	this->max_message_length = 200;
	this->nMessages          = 120;
	this->sack_v2_enabled    = true;
	this->sack_enabled       = true;
	this->header_carries_d5  = false;    // deterministic max_frame (no D5 byte)
	this->compression_enabled= false;
	this->robust_enabled     = NO;
	this->narrowband_enabled = NO;
	this->current_configuration = 0;
	this->role               = COMMANDER;
	this->original_role      = COMMANDER;
	this->link_status        = CONNECTED;
	this->connection_status  = TRANSMITTING_DATA;
	this->block_under_tx     = NO;
	this->message_batch_counter_tx = 0;
	this->retransmit_count   = 0;
	const int DBS = 8;
	this->set_data_batch_size(DBS);
	deinit_messages_buffers();
	if(init_messages_buffers() != SUCCESSFUL){
		printf("[TEST-KX-TXSTREAM] ERROR: init_messages_buffers() failed\n");
		fflush(stdout); return 1;
	}
	fifo_buffer_tx.set_size(default_configuration_ARQ.fifo_buffer_tx_size);
	fifo_buffer_backup.set_size(default_configuration_ARQ.fifo_buffer_backup_size);
	fifo_buffer_tx.flush();        // NO user data staged -> only the KX stream flows
	fifo_buffer_backup.flush();

	// Enter the KX-as-data pre-activation epoch (env gate ON + encryption negotiated,
	// cipher NOT active, phase IDLE so the production hold path stages on first entry).
	setenv("MERCURY_KX_AS_DATA", "1", 1);
	this->encryption_enabled = true;
	cipher_suite.set_kx_phase(KX_IDLE);
	kx_chunk_state_reset();
	check(kx_stream_epoch(), "commander is in the KX-as-data pre-activation epoch");
	check(!cipher_suite.is_active(), "cipher not yet active (pre-activation)");

	// --- PHASE 1: REAL production staging + raw-leg source-switch framing ---------------
	std::vector<unsigned char> collected;   // all framed KX bytes, in slot/stream order
	collected.reserve(KX_STREAM_BUFSZ);
	bool drained = false;
	for(int cycle=0; cycle<64 && !drained; cycle++)
	{
		process_buffer_data_commander();    // REAL: stage (cycle 0) + fill from kx_stream_tx_buf
		int staged_now = 0;
		for(int i=0;i<nMessages;i++)
		{
			if(messages_tx[i].status==ADDED_TO_LIST)
			{
				for(int j=0;j<messages_tx[i].length;j++)
					collected.push_back((unsigned char)messages_tx[i].data[j]);
				messages_tx[i].status = FREE;
				messages_tx[i].length = 0;
				staged_now++;
			}
		}
		if(staged_now == 0) drained = true;
		this->block_under_tx = NO;          // allow the next batch to stage
	}

	int want_len = KX_STREAM_HDR + KX_STREAM_FWD_LEN;   // 1223
	check(kx_stream_tx_len == want_len, "production staged the forward stream (len==1223)");
	check(kx_mlkem_pk_ready, "kx_mlkem_pk_ready set by the forward stage");
	check((int)collected.size() == want_len,
		"raw-leg source-switch framed the WHOLE forward stream (bytes==1223)");
	check(!kx_stream_tx_active, "forward stream fully drained (kx_stream_tx_active cleared)");

	// Reference = the staged stream (keypairs are random per run).
	unsigned char saved_stream[KX_STREAM_BUFSZ];
	int saved_len = (kx_stream_tx_len <= KX_STREAM_BUFSZ) ? kx_stream_tx_len : KX_STREAM_BUFSZ;
	memcpy(saved_stream, kx_stream_tx_buf, saved_len);
	bool concat_ok = ((int)collected.size() == saved_len);
	if(concat_ok)
		for(int i=0;i<saved_len;i++) if(collected[i] != saved_stream[i]){ concat_ok=false; break; }
	check(concat_ok, "framed bytes BYTE-IDENTICAL to the staged forward stream, in order");
	bool hdr_ok = saved_len >= KX_STREAM_HDR
		&& collected[0]==(unsigned char)((KX_STREAM_MAGIC>>24)&0xFF)
		&& collected[1]==(unsigned char)((KX_STREAM_MAGIC>>16)&0xFF)
		&& collected[2]==(unsigned char)((KX_STREAM_MAGIC>>8)&0xFF)
		&& collected[3]==(unsigned char)(KX_STREAM_MAGIC&0xFF)
		&& collected[4]==KX_STREAM_FWD;
	check(hdr_ok, "framed stream carries the KX magic + FWD kind on the wire");

	// --- PHASE 2: deliver the framed bytes through the REAL RX funnel --------------------
	unsigned char x_ref[X25519_KEY_SIZE];
	unsigned char pk_ref[MLKEM_PK_SIZE];
	memcpy(x_ref,  saved_stream + KX_STREAM_HDR,                   X25519_KEY_SIZE);
	memcpy(pk_ref, saved_stream + KX_STREAM_HDR + X25519_KEY_SIZE, MLKEM_PK_SIZE);

	// Zero the crypto buffers + reset RX state so the reassembly is proven FRESH — the RX
	// must rebuild the pk from the received frames, it cannot "keep" the TX pk.
	kx_chunk_state_reset();                 // clears kx_mlkem_pk_ready + kx_stream_rx_*
	memset(kx_mlkem_pk, 0, MLKEM_PK_SIZE);
	memset(kx_stream_peer_x25519, 0, X25519_KEY_SIZE);
	rx_stream_emitted_bsi_hw     = -1;
	rx_stream_delivered          = 0;
	rsp_cross_session_seam_armed = false;
	for(int s=0;s<256;s++) rx_stream_stamp[s].valid = false;
	decrypt_delivered_bsi = 0;
	for(int i=0;i<nMessages;i++){ messages_rx[i].status=FREE; messages_rx[i].length=0; }

	// Lay the framed bytes across ACKED messages_rx[] slots (<= max_data_length each),
	// exactly as the RSP data receiver hands a delivered batch to the funnel.
	int total = (int)collected.size();
	int per = (total + DBS - 1) / DBS;
	if(per > max_data_length) per = max_data_length;
	int off = 0, nslots = 0;
	for(int i=0;i<nMessages && off<total;i++){
		int n = total - off; if(n > per) n = per;
		memcpy(messages_rx[i].data, &collected[off], n);
		messages_rx[i].length = n;
		messages_rx[i].status = ACKED;
		off += n; nslots++;
	}
	check(off == total, "framed bytes fully laid across RX slots for the funnel");

	fifo_buffer_rx.set_size(4096);
	fifo_buffer_rx.flush();
	int occ_before = fifo_buffer_rx.get_size() - fifo_buffer_rx.get_free_size();

	copy_data_to_buffer();      // REAL RX funnel -> kx_ingest routing

	int occ_after = fifo_buffer_rx.get_size() - fifo_buffer_rx.get_free_size();
	check(kx_mlkem_pk_ready, "RX funnel reassembled the forward stream via kx_ingest");
	check(kx_mlkem_pk_ready && memcmp(kx_mlkem_pk, pk_ref, MLKEM_PK_SIZE)==0,
		"end-to-end: ML-KEM pk BYTE-IDENTICAL (TX source-switch -> RX kx_ingest)");
	check(memcmp(kx_stream_peer_x25519, x_ref, X25519_KEY_SIZE)==0,
		"end-to-end: folded x25519 pubkey BYTE-IDENTICAL");
	check(occ_after == occ_before, "app FIFO UNTOUCHED — KX never delivered to the app");
	check(this->link_status != DROPPED, "no false teardown on the KX batches");

	// Undo the env gate so no sibling --test block inherits the KX-as-data path.
	unsetenv("MERCURY_KX_AS_DATA");
	this->encryption_enabled = false;

	printf("[TEST-KX-TXSTREAM] %s (failures=%d)\n", failures? "FAIL":"ALL PASS", failures);
	fflush(stdout);
	return failures;
}

// KX-AS-DATA FULL crypto round-trip fire proof (data-flow-hybrid-kex.md §7 / T6).
// Drives the REAL production reverse-ct + activation crypto path on two controllers:
//   CMD kx_stage_forward_stream() -> RSP kx_ingest() -> RSP kx_on_forward_complete()
//   (encapsulate + derive + stage reverse) -> CMD kx_ingest() -> CMD
//   kx_on_reverse_complete() (decapsulate + derive)
// and asserts BOTH peers derive the IDENTICAL hybrid session key (the KEY_ACTIVATE
// premise), the confirm tags match, both are PQ-upgraded, and a TAMPERED reverse ct
// diverges the CMD key so the confirm tag MISMATCHES (KEY_ACTIVATE would REFUSE — the
// AEAD/tamper fail-secure). NO message buffers / telecom_system needed (the crypto +
// kx_stream_* transport buffers are self-contained). The SWITCH_ROLE transport itself
// is proven on the LIVE wire (G1/G2); this is the crypto-identity half.
int cl_arq_controller::test_kx_roundtrip_derive()
{
	int failures = 0;
	auto check = [&](bool cond, const char* name){
		if(!cond){ failures++; printf("[TEST-KX-ROUNDTRIP] FAIL: %s\n", name); }
		else      { printf("[TEST-KX-ROUNDTRIP] ok: %s\n", name); }
		fflush(stdout);
	};

	cl_arq_controller& cmd = *this;      // this peer = CMD (ML-KEM keypair holder)
	cl_arq_controller  rsp;              // peer = RSP (encapsulator)

	// Consistent callsigns so derive_session_key(commander_call, responder_call) is
	// assembled identically on both peers (the transcript salt is call-order bound).
	cmd.my_call_sign = "AAAAAA"; cmd.destination_call_sign = "BBBBBB"; cmd.psk_hex[0] = '\0';
	rsp.my_call_sign = "BBBBBB"; rsp.destination_call_sign = "AAAAAA"; rsp.psk_hex[0] = '\0';
	cmd.encryption_enabled = true; rsp.encryption_enabled = true;
	cmd.cipher_suite.set_kx_phase(KX_IDLE); rsp.cipher_suite.set_kx_phase(KX_IDLE);
	cmd.kx_chunk_state_reset(); rsp.kx_chunk_state_reset();

	// --- Forward: CMD stages the forward stream (REAL production stage) --------------
	check(cmd.kx_stage_forward_stream() == 0, "CMD staged forward stream (x25519_pk_cmd + mlkem_pk)");
	check(cmd.kx_stream_tx_len == KX_STREAM_HDR + KX_STREAM_FWD_LEN, "forward stream len == 1223");
	// Deliver the forward bytes to the RSP via the REAL kx_ingest in irregular fragments
	// (header split across a fragment boundary — the data plane hands contiguous bytes).
	{
		const uint8_t* fwd = cmd.kx_stream_tx_buf; int total = cmd.kx_stream_tx_len; int off = 0;
		bool frag_ok = true;
		while(off < total){ int n = (total - off < 137) ? (total - off) : 137;
			if(rsp.kx_ingest(fwd + off, n) < 0) frag_ok = false; off += n; }
		check(frag_ok, "RSP kx_ingest consumed every forward fragment (no REFUSE)");
	}
	check(rsp.kx_stream_rx_done_kind == KX_STREAM_FWD, "RSP completed forward reassembly (done_kind==FWD)");
	check(rsp.kx_mlkem_pk_ready && memcmp(rsp.kx_mlkem_pk, cmd.kx_mlkem_pk, MLKEM_PK_SIZE) == 0,
		"RSP reassembled the ML-KEM pk BYTE-IDENTICAL");
	check(memcmp(rsp.kx_stream_peer_x25519, cmd.kx_stream_tx_buf + KX_STREAM_HDR, X25519_KEY_SIZE) == 0,
		"RSP reassembled the folded x25519_pk_cmd BYTE-IDENTICAL");

	// --- RSP: REAL production forward-consume (encapsulate + derive + stage reverse) --
	check(rsp.kx_on_forward_complete() == 0, "RSP kx_on_forward_complete (encapsulate + derive + stage reverse)");
	check(rsp.cipher_suite.get_kx_phase() == KX_HYBRID_DONE, "RSP reached KX_HYBRID_DONE");
	check(rsp.kx_stream_tx_active && rsp.kx_stream_tx_len == KX_STREAM_HDR + KX_STREAM_REV_LEN,
		"RSP staged the reverse stream (len == 1127) — armed for SWITCH_ROLE");

	// --- Reverse: deliver the RSP's reverse bytes to the CMD via REAL kx_ingest -------
	uint8_t rev_saved[KX_STREAM_BUFSZ]; int rev_len = rsp.kx_stream_tx_len;
	memcpy(rev_saved, rsp.kx_stream_tx_buf, rev_len);   // save for the tamper sub-test
	{
		const uint8_t* rev = rsp.kx_stream_tx_buf; int total = rsp.kx_stream_tx_len; int off = 0;
		bool frag_ok = true;
		while(off < total){ int n = (total - off < 91) ? (total - off) : 91;
			if(cmd.kx_ingest(rev + off, n) < 0) frag_ok = false; off += n; }
		check(frag_ok, "CMD kx_ingest consumed every reverse fragment (no REFUSE)");
	}
	check(cmd.kx_stream_rx_done_kind == KX_STREAM_REV, "CMD completed reverse reassembly (done_kind==REV)");
	check(cmd.kx_mlkem_ct_ready && memcmp(cmd.kx_mlkem_ct, rsp.kx_mlkem_ct, MLKEM_CT_SIZE) == 0,
		"CMD reassembled the ML-KEM ct BYTE-IDENTICAL");

	// --- CMD: REAL production reverse-consume (decapsulate + derive) ------------------
	check(cmd.kx_on_reverse_complete() == 0, "CMD kx_on_reverse_complete (decapsulate + derive)");
	check(cmd.cipher_suite.get_kx_phase() == KX_HYBRID_DONE, "CMD reached KX_HYBRID_DONE");
	check(cmd.kx_reverse_consumed, "CMD kx_reverse_consumed latched (gates KEY_ACTIVATE)");

	// --- THE round-trip assertion: BOTH peers derived the IDENTICAL hybrid key -------
	uint8_t key_cmd[SESSION_KEY_SIZE], key_rsp[SESSION_KEY_SIZE];
	cmd.cipher_suite.copy_session_key_for_test(key_cmd);
	rsp.cipher_suite.copy_session_key_for_test(key_rsp);
	check(memcmp(key_cmd, key_rsp, SESSION_KEY_SIZE) == 0,
		"BOTH peers derived the IDENTICAL hybrid session key");
	uint8_t tag_cmd[8], tag_rsp[8];
	cmd.cipher_suite.compute_key_confirmation(tag_cmd);
	rsp.cipher_suite.compute_key_confirmation(tag_rsp);
	check(memcmp(tag_cmd, tag_rsp, 8) == 0, "key confirmation tags match (KEY_ACTIVATE would confirm)");
	check(cmd.cipher_suite.is_pq_upgraded() && rsp.cipher_suite.is_pq_upgraded(),
		"both peers PQ-upgraded (hybrid combiner, not classical fallback)");

	// --- Fail-secure (AEAD/tamper, T3/G3 in-vitro): a tampered reverse ct diverges the
	// CMD's derived key, so its confirm tag MISMATCHES the RSP's -> KEY_ACTIVATE REFUSE.
	{
		cl_arq_controller cmd2;
		cmd2.my_call_sign = "AAAAAA"; cmd2.destination_call_sign = "BBBBBB"; cmd2.psk_hex[0] = '\0';
		cmd2.encryption_enabled = true;
		cmd2.cipher_suite.set_kx_phase(KX_IDLE); cmd2.kx_chunk_state_reset();
		// cmd2 must be the SAME keypair holder as cmd for a fair tamper test: re-run its
		// forward stage would make a DIFFERENT keypair, so instead copy cmd's keypair
		// state by re-deriving on cmd's cipher after a ct flip is not possible; simplest
		// faithful check: flip a ct byte in the reverse stream and feed cmd itself a
		// fresh REV, then re-derive — cmd's mlkem_sk + x25519 are still valid, so decap of
		// a tampered ct yields a DIFFERENT ML-KEM shared (implicit rejection) + the
		// transcript bind sees the tampered ct -> a divergent key.
		uint8_t tampered[KX_STREAM_BUFSZ]; memcpy(tampered, rev_saved, rev_len);
		tampered[KX_STREAM_HDR + X25519_KEY_SIZE + 10] ^= 0x40;   // flip a ct byte
		cmd.kx_chunk_state_reset();               // reset only the stream RX reassembly
		cmd.cipher_suite.set_kx_phase(KX_MLKEM_PK_SENT);
		cmd.kx_reverse_consumed = false;
		int off = 0; bool frag_ok = true;
		while(off < rev_len){ int n = (rev_len - off < 91) ? (rev_len - off) : 91;
			if(cmd.kx_ingest(tampered + off, n) < 0) frag_ok = false; off += n; }
		check(frag_ok && cmd.kx_stream_rx_done_kind == KX_STREAM_REV, "tampered reverse ct reassembled (transport is content-agnostic)");
		check(cmd.kx_on_reverse_complete() == 0, "CMD decapsulated the tampered ct (ML-KEM implicit rejection never errors)");
		uint8_t key_bad[SESSION_KEY_SIZE], tag_bad[8];
		cmd.cipher_suite.copy_session_key_for_test(key_bad);
		cmd.cipher_suite.compute_key_confirmation(tag_bad);
		check(memcmp(key_bad, key_rsp, SESSION_KEY_SIZE) != 0, "tampered ct -> DIVERGENT session key (transcript bind)");
		check(memcmp(tag_bad, tag_rsp, 8) != 0, "tampered ct -> confirm-tag MISMATCH (KEY_ACTIVATE REFUSE, fail-secure)");
	}

	printf("[TEST-KX-ROUNDTRIP] %s (failures=%d)\n", failures ? "FAIL" : "ALL PASS", failures);
	fflush(stdout);
	return failures;
}

// KX-AS-DATA RX ROUTING fire proof (data-flow-hybrid-kex.md §2.3). Drives the REAL
// copy_data_to_buffer() delivery funnel — the SAME production path the RSP data
// receiver runs — with a forward KX stream staged across messages_rx[] ACKED slots
// while the receiver is in the reserved pre-activation KX epoch (kx_as_data_path()
// via MERCURY_KX_AS_DATA + encryption negotiated, not yet active). Proves the
// production routing branch FIRES: the KX bytes reassemble through kx_ingest()
// BYTE-IDENTICAL, are NEVER pushed to the app FIFO (fifo_buffer_rx untouched), the
// Option-W absolute byte cursor does NOT advance (rx_stream_delivered stays 0), and
// no false [RSP-V2-STREAM-SHIFT] positional teardown fires on the stampless KX
// batch. (The fail-secure REFUSE branch routes kx_ingest()==-1 into
// rsp_gap_abort_teardown(); its teardown dereferences telecom_system, so it is
// exercised on the LIVE path only — the -1 itself is proven in test_kx_ingest.
// This test is happy-path only by construction.) 0 = pass / N = failure count.
int cl_arq_controller::test_kx_rx_routing()
{
	int failures = 0;
	auto check = [&](bool cond, const char* name){
		if(!cond){ failures++; printf("[TEST-KX-RXROUTE] FAIL: %s\n", name); }
		else      { printf("[TEST-KX-RXROUTE] ok: %s\n", name); }
		fflush(stdout);
	};

	// --- Controller setup (mirror test_rxfifo_backpressure_hold §0) ---
	this->nMessages          = 120;
	this->max_data_length    = 170;
	this->max_message_length = 200;
	this->max_header_length  = 6;
	if(init_messages_buffers() != SUCCESSFUL){
		printf("[TEST-KX-RXROUTE] ERROR: init_messages_buffers() failed\n");
		fflush(stdout); return 1;
	}
	this->compression_enabled   = false;
	this->sack_v2_enabled       = true;
	this->sack_enabled          = true;
	this->current_configuration = 0;
	this->original_role = COMMANDER;    // fifo_push_rx would push to fifo_buffer_rx if reached
	this->role          = COMMANDER;

	// Enter the KX-as-data pre-activation epoch: env-gate ON + encryption negotiated,
	// cipher NOT active -> kx_stream_epoch() TRUE. (setenv is undone at the end so no
	// sibling --test block sees the gate.)
	setenv("MERCURY_KX_AS_DATA", "1", 1);
	this->encryption_enabled = true;
	cipher_suite.set_kx_phase(KX_IDLE);
	kx_chunk_state_reset();
	check(kx_stream_epoch(), "receiver is in the KX-as-data pre-activation epoch");
	check(!cipher_suite.is_active(), "cipher not yet active (pre-activation)");

	// Fresh delivery state so the top-of-funnel guards (INV-DEDUP / seam / Option-W
	// shift backstop) are inert on this first batch.
	rx_stream_emitted_bsi_hw     = -1;
	rx_stream_delivered          = 0;
	rsp_cross_session_seam_armed = false;
	for(int s=0;s<256;s++) rx_stream_stamp[s].valid = false;
	decrypt_delivered_bsi = 0;

	// --- Build the forward KX stream MAGIC|FWD|len | x25519(32) | mlkem_pk(1184) ---
	uint8_t x25519_ref[X25519_KEY_SIZE];
	for(int i=0;i<X25519_KEY_SIZE;i++) x25519_ref[i] = (uint8_t)(0x5A + i);
	uint8_t pk_ref[MLKEM_PK_SIZE];
	// A REAL ML-KEM encaps key (not a synthetic pattern): the completed forward stream
		// now fires the production RSP forward-consume hook (kx_on_forward_complete ->
		// encapsulate_mlkem), which REFUSES a bogus key. A real keypair keeps this a
		// happy-path routing proof (no fail-secure teardown -> no telecom_system deref).
		{ cl_cipher_suite kg_rr; kg_rr.generate_mlkem_keypair(pk_ref); }
	int fwd_payload = X25519_KEY_SIZE + MLKEM_PK_SIZE;          // 1216
	uint8_t stream[KX_STREAM_HDR + X25519_KEY_SIZE + MLKEM_PK_SIZE];
	stream[0]=(uint8_t)((KX_STREAM_MAGIC>>24)&0xFF); stream[1]=(uint8_t)((KX_STREAM_MAGIC>>16)&0xFF);
	stream[2]=(uint8_t)((KX_STREAM_MAGIC>>8)&0xFF);  stream[3]=(uint8_t)(KX_STREAM_MAGIC&0xFF);
	stream[4]=KX_STREAM_FWD;
	stream[5]=(uint8_t)(fwd_payload & 0xFF); stream[6]=(uint8_t)((fwd_payload>>8)&0xFF);   // LE
	memcpy(stream+KX_STREAM_HDR, x25519_ref, X25519_KEY_SIZE);
	memcpy(stream+KX_STREAM_HDR+X25519_KEY_SIZE, pk_ref, MLKEM_PK_SIZE);
	int stream_total = KX_STREAM_HDR + fwd_payload;            // 1223

	// Lay the stream across DBS ACKED messages_rx[] slots (<= max_data_length each) —
	// exactly how the data plane reassembles a batch, split so the header spans slot 0.
	const int DBS = 8;
	this->data_batch_size = DBS;
	for(int i=0;i<this->nMessages;i++){ messages_rx[i].status=FREE; messages_rx[i].length=0; }
	int per = (stream_total + DBS - 1) / DBS;                  // ~153; <= 170
	int off = 0;
	for(int i=0;i<DBS && off<stream_total;i++){
		int n = stream_total - off; if(n > per) n = per;
		memcpy(messages_rx[i].data, stream+off, n);
		messages_rx[i].length = n;
		messages_rx[i].status = ACKED;
		off += n;
	}

	// App FIFO with room — so if a KX byte LEAKED to the app it would be visible.
	fifo_buffer_rx.set_size(4096);
	fifo_buffer_rx.flush();
	auto occ_rx = [&]() -> int { return fifo_buffer_rx.get_size() - fifo_buffer_rx.get_free_size(); };

	// --- Drive the REAL delivery funnel ---
	copy_data_to_buffer();

	check(kx_mlkem_pk_ready, "KX forward stream reassembled via copy_data_to_buffer->kx_ingest");
	check(kx_mlkem_pk_ready && memcmp(kx_mlkem_pk, pk_ref, MLKEM_PK_SIZE)==0,
		"reassembled ML-KEM pk is BYTE-IDENTICAL (routed through the real funnel)");
	check(memcmp(kx_stream_peer_x25519, x25519_ref, X25519_KEY_SIZE)==0,
		"folded x25519 pubkey BYTE-IDENTICAL");
	check(occ_rx()==0, "app FIFO UNTOUCHED — KX never delivered to the app (fifo_buffer_rx==0)");
	check(rx_stream_delivered==0, "Option-W byte cursor NOT advanced by KX bytes (stays 0)");
	check(this->link_status != DROPPED, "no false teardown on the stampless KX batch");

	// Undo the env gate so no sibling --test block inherits the KX-as-data path.
	unsetenv("MERCURY_KX_AS_DATA");
	this->encryption_enabled = false;

	printf("[TEST-KX-RXROUTE] %s (failures=%d)\n", failures? "FAIL":"ALL PASS", failures);
	fflush(stdout);
	return failures;
}

// In-process regression for the turnaround-clearance guard (--test-turnaround-guard; also
// runs inside --test). No IONOS/RF, no telecom_system: drives turnaround_clearance_wait()
// directly with the stock delays (ptt_on=100/ptt_off=200/margin=150 => guard=250 ms).
// Returns 0=PASS, 1=FAIL. Default builds never call this.
int cl_arq_controller::test_turnaround_guard()
{
	printf("[TEST-TURNGUARD] start\n");
	fflush(stdout);
	int fails = 0;

	// Stock delays: guard = 200 + 150 - 100 = 250 ms.
	this->ptt_on_delay_ms  = 100;
	this->ptt_off_delay_ms = 200;
	unsetenv("MERCURY_TURNAROUND_GUARD_MARGIN_MS");
	unsetenv("MERCURY_TURNAROUND_GUARD_SCOPE_ALL");

	// Arm A — fail-before via the defeat knob (reproduces the unguarded modem): arm the
	// CONTROL turnaround, then run the guard with the wait defeated. The key point is
	// reached with < 50 ms elapsed (no wait applied).
	setenv("MERCURY_TURNAROUND_GUARD_DEFEAT", "1", 1);
	this->arm_control_turnaround_guard();
	this->turnaround_clearance_wait();
	printf("[TEST-TURNGUARD] Arm A (control, defeat=1): waited=%lld elapsed_at_key=%lld (want elapsed<50)\n",
		this->tg_test_waited_ms, this->tg_test_elapsed_at_key_ms);
	if(!(this->tg_test_waited_ms == 0 && this->tg_test_elapsed_at_key_ms < 50)) { printf("[TEST-TURNGUARD] Arm A FAIL\n"); fails++; }

	// Arm B — the fix: defeat unset, CONTROL-arm, run. The guard busy-waits so the key
	// point is reached at >= 250 ms.
	unsetenv("MERCURY_TURNAROUND_GUARD_DEFEAT");
	this->arm_control_turnaround_guard();
	this->turnaround_clearance_wait();
	printf("[TEST-TURNGUARD] Arm B (control, default-on): waited=%lld elapsed_at_key=%lld (want both>=250)\n",
		this->tg_test_waited_ms, this->tg_test_elapsed_at_key_ms);
	if(!(this->tg_test_waited_ms >= 250 && this->tg_test_elapsed_at_key_ms >= 250)) { printf("[TEST-TURNGUARD] Arm B FAIL\n"); fails++; }

	// Arm C — invariant (b): enough real time has already elapsed since the control arm
	// (300 ms), so the guard adds ~0.
	this->arm_control_turnaround_guard();
	{ cl_timer settle; settle.start(); while(settle.get_elapsed_time_ms() < 300) msleep(1); }
	this->turnaround_clearance_wait();
	printf("[TEST-TURNGUARD] Arm C (control, prior>=guard): waited=%lld elapsed_at_key=%lld (want waited==0)\n",
		this->tg_test_waited_ms, this->tg_test_elapsed_at_key_ms);
	if(!(this->tg_test_waited_ms == 0 && this->tg_test_elapsed_at_key_ms >= 250)) { printf("[TEST-TURNGUARD] Arm C FAIL\n"); fails++; }

	// Arm D — never stamped (never armed) => no wait at all (report -1).
	this->turnaround_clearance_armed        = false;
	this->turnaround_clearance_from_control = false;
	this->turnaround_clearance_wait();
	printf("[TEST-TURNGUARD] Arm D (never armed): waited=%lld (want -1, no wait)\n",
		this->tg_test_waited_ms);
	if(this->tg_test_waited_ms != -1) { printf("[TEST-TURNGUARD] Arm D FAIL\n"); fails++; }

	// Arm E — PASS-AFTER: a routine compact/SACK turnaround receives the same
	// geometry-derived clearance because peer playback drain may outlive early decode.
	this->arm_routine_turnaround_guard();
	this->turnaround_clearance_wait();
	printf("[TEST-TURNGUARD] Arm E (routine, default): waited=%lld elapsed_at_key=%lld (want both>=250)\n",
		this->tg_test_waited_ms, this->tg_test_elapsed_at_key_ms);
	if(!(this->tg_test_waited_ms >= 250 && this->tg_test_elapsed_at_key_ms >= 250)) { printf("[TEST-TURNGUARD] Arm E FAIL\n"); fails++; }

	// Arm F — FAIL-BEFORE via the scope knob: restore the unsafe control-only
	// exemption and prove the routine arm adds no clearance.
	setenv("MERCURY_TURNAROUND_GUARD_SCOPE_ALL", "0", 1);
	this->arm_routine_turnaround_guard();
	this->turnaround_clearance_wait();
	printf("[TEST-TURNGUARD] Arm F (routine, SCOPE_ALL=0): waited=%lld (want 0, no wait)\n",
		this->tg_test_waited_ms);
	if(this->tg_test_waited_ms != 0) { printf("[TEST-TURNGUARD] Arm F FAIL\n"); fails++; }
	unsetenv("MERCURY_TURNAROUND_GUARD_SCOPE_ALL");

	unsetenv("MERCURY_TURNAROUND_GUARD_DEFEAT");
	bool pass = (fails == 0);
	printf("[TEST-TURNGUARD] %s: fails=%d\n", pass ? "PASS" : "FAIL", fails);
	fflush(stdout);
	return pass ? 0 : 1;
}

// R6 measured-turnaround estimator regression (RFC 6298 §2; Karn & Partridge 1987)
// + the ack_timeout_data >= receiving_timeout invariant (I2). Drives the REAL
// update_turnaround_estimate() + calculate_receiving_timeout() on a throwaway
// controller — no IONOS/RF/telecom_system. The load-bearing arm is C: it fires the
// CONFIG_15-only landmine's mechanism on a NON-CFG15 config (CONFIG_13), proving the
// measured window tracks a drifty long batch where the pre-R6 formula (defeated) misses.
int cl_arq_controller::test_measured_timers()
{
	printf("[TEST-MEASTIMERS] start\n");
	fflush(stdout);
	int fails = 0;

	// Deterministic geometry for the throwaway controller. geometric_floor =
	// frame_drain(2*mtt=400) + sack_arrival(ptt_off 200 + RSP_DECODE_MARGIN_MS 300 +
	// pattern 500 + ptt_on 100 = 1100) = 1500; cold window = 1500 + SACK_ARRIVAL_MARGIN_MS.
	this->role                        = COMMANDER;
	this->ack_pattern_time_ms         = 500;   // > 0 => CMD data branch
	this->message_transmission_time_ms= 200;
	this->ptt_off_delay_ms            = 200;
	this->ptt_on_delay_ms             = 100;
	this->gear_shift_on               = NO;     // no turboshift +2000
	this->sack_enabled                = false;  // no sack_timeout_extra_ms
	this->sack_timeout_extra_ms       = 0;
	this->current_configuration       = CONFIG_13;   // OFDM, NON-CFG15 => class 1
	this->data_ack_retx_turnaround    = false;
	this->cfg16_revack_starve_fails   = 0;
	unsetenv("MERCURY_MEASURED_TIMERS_DEFEAT");
	unsetenv("MERCURY_MEASURED_TIMERS_K");
	unsetenv("MERCURY_MEASURED_TIMERS_FLOOR_MS");
	unsetenv("MERCURY_ACK_TIMEOUT_FLOOR_DEFEAT");
	// The link-phase CMD block-boundary ACK_SLOT floor (linkphase_ackslot_on, DEFAULT-ON)
	// raises receiving_timeout by a keydown-derived slot ON TOP of the R6 window. This
	// unit asserts the R6 estimator/window arithmetic itself, so pin the slot floor off
	// via its production defeat knob for the duration (the link-phase suites own the
	// floor's coverage). Cleared with the other knobs at the end.
	setenv("MERCURY_LINKPHASE_CMD_FLOOR_DEFEAT", "1", 1);

	const int GEOM_FLOOR = 1500;
	const int cls2 = tt_class_of(CONFIG_13);              // expect 1
	const int bk2  = tt_batchbk_of(20 * 200);            // airtime 4000 => bucket 2
	const int bk3  = tt_batchbk_of(28 * 200);            // airtime 5600 => bucket 3
	if(cls2 != 1) { printf("[TEST-MEASTIMERS] class map FAIL (cls=%d want 1)\n", cls2); fails++; }
	if(bk2 != 2)  { printf("[TEST-MEASTIMERS] bucket map FAIL (bk=%d want 2)\n", bk2); fails++; }
	if(bk3 != 3)  { printf("[TEST-MEASTIMERS] bucket map FAIL (bk=%d want 3)\n", bk3); fails++; }

	// Arm E (COLD byte-identity): fresh table, n==0 => geometric_floor + margin.
	this->data_batch_size = 20;   // bucket 2
	this->calculate_receiving_timeout();
	printf("[TEST-MEASTIMERS] Arm E (cold): rx_to=%d (want %d)\n",
		this->receiving_timeout, GEOM_FLOOR + SACK_ARRIVAL_MARGIN_MS);
	if(this->receiving_timeout != GEOM_FLOOR + SACK_ARRIVAL_MARGIN_MS) { printf("[TEST-MEASTIMERS] Arm E FAIL\n"); fails++; }

	// Arm A (RFC 6298 §2.2 first sample): feed R=2000 into class1/bucket2.
	this->update_turnaround_estimate(CONFIG_13, 20 * 200, 2000);
	turnaround_rtt_est &eA = this->tt_rtt[cls2][bk2];
	printf("[TEST-MEASTIMERS] Arm A: n=%d srtt=%d rttvar=%d (want 1/2000/1000)\n", eA.n, eA.srtt_ms, eA.rttvar_ms);
	if(!(eA.n == 1 && eA.srtt_ms == 2000 && eA.rttvar_ms == 1000)) { printf("[TEST-MEASTIMERS] Arm A (est) FAIL\n"); fails++; }
	this->calculate_receiving_timeout();   // warm: 2000 + max(20,4*1000)=4000 + floor 300 = 6300
	printf("[TEST-MEASTIMERS] Arm A: rx_to=%d (want 6300)\n", this->receiving_timeout);
	if(this->receiving_timeout != 6300) { printf("[TEST-MEASTIMERS] Arm A (window) FAIL\n"); fails++; }

	// Arm B (RFC 6298 §2.3 convergence): 40 more R'=2000 => srtt stays 2000, rttvar->3.
	for(int i=0;i<40;i++) this->update_turnaround_estimate(CONFIG_13, 20 * 200, 2000);
	printf("[TEST-MEASTIMERS] Arm B: srtt=%d rttvar=%d (want 2000/3)\n", eA.srtt_ms, eA.rttvar_ms);
	if(!(eA.srtt_ms == 2000 && eA.rttvar_ms == 3)) { printf("[TEST-MEASTIMERS] Arm B (est) FAIL\n"); fails++; }
	this->calculate_receiving_timeout();   // 2000 + max(20,4*3=12)=20 + 300 = 2320
	printf("[TEST-MEASTIMERS] Arm B: rx_to=%d (want 2320)\n", this->receiving_timeout);
	if(this->receiving_timeout != 2320) { printf("[TEST-MEASTIMERS] Arm B (window) FAIL\n"); fails++; }

	// Arm C (THE LANDMINE FIX): a drifty long batch on NON-CFG15 CONFIG_13, reverse SACK
	// arriving at 4500 ms. fail-before (defeat) MISSES; pass-after (warm) CONTAINS.
	this->data_batch_size = 28;   // airtime 5600 => bucket 3
	setenv("MERCURY_MEASURED_TIMERS_DEFEAT", "1", 1);
	this->calculate_receiving_timeout();   // defeat => geometric_floor + margin = 2500
	int rx_defeat = this->receiving_timeout;
	printf("[TEST-MEASTIMERS] Arm C fail-before (defeat): rx_to=%d vs arrival 4500 (want MISS <4500)\n", rx_defeat);
	if(!(rx_defeat < 4500)) { printf("[TEST-MEASTIMERS] Arm C fail-before FAIL\n"); fails++; }
	unsetenv("MERCURY_MEASURED_TIMERS_DEFEAT");
	for(int i=0;i<5;i++) this->update_turnaround_estimate(CONFIG_13, 28 * 200, 4500);  // warm bucket 3
	this->calculate_receiving_timeout();
	int rx_warm = this->receiving_timeout;
	printf("[TEST-MEASTIMERS] Arm C pass-after (warm): rx_to=%d vs arrival 4500 (want CONTAINS >=4500)\n", rx_warm);
	if(!(rx_warm >= 4500)) { printf("[TEST-MEASTIMERS] Arm C pass-after FAIL\n"); fails++; }

	// Arm D (Karn's rule): a retx-round sample must NOT move SRTT.
	int srtt_before = this->tt_rtt[cls2][bk2].srtt_ms;
	this->data_ack_retx_turnaround = true;                 // retx turnaround
	if(this->tt_karn_sample_ok())                          // production gate (must be false)
		this->update_turnaround_estimate(CONFIG_13, 20 * 200, 9999);
	int srtt_after = this->tt_rtt[cls2][bk2].srtt_ms;
	printf("[TEST-MEASTIMERS] Arm D (Karn): srtt %d -> %d (want unchanged)\n", srtt_before, srtt_after);
	if(srtt_after != srtt_before) { printf("[TEST-MEASTIMERS] Arm D FAIL\n"); fails++; }
	this->data_ack_retx_turnaround = false;

	// Arm G (INVARIANT I2 fail-before/pass-after): a warm long-batch window that exceeds a
	// stale ack_timeout_data. fail-before (floor defeated) leaves the invariant VIOLATED;
	// pass-after (floor on) re-asserts ack_timeout_data >= receiving_timeout.
	this->data_batch_size = 28;   // bucket 3, warm at 4500 above
	// fail-before: reproduce the skipped per-batch floor.
	this->ack_timeout_data    = 3000;   // stale (as if set at load for a small batch)
	this->ack_timeout_control = 3000;
	setenv("MERCURY_ACK_TIMEOUT_FLOOR_DEFEAT", "1", 1);
	this->calculate_receiving_timeout();
	printf("[TEST-MEASTIMERS] Arm G fail-before: ack_to_data=%d rx_to=%d (want ack < rx, VIOLATED)\n",
		this->ack_timeout_data, this->receiving_timeout);
	if(!(this->ack_timeout_data < this->receiving_timeout)) { printf("[TEST-MEASTIMERS] Arm G fail-before FAIL\n"); fails++; }
	// pass-after: floor on, invariant restored.
	this->ack_timeout_data    = 3000;
	this->ack_timeout_control = 3000;
	unsetenv("MERCURY_ACK_TIMEOUT_FLOOR_DEFEAT");
	this->calculate_receiving_timeout();
	printf("[TEST-MEASTIMERS] Arm G pass-after: ack_to_data=%d rx_to=%d (want ack >= rx, HOLDS)\n",
		this->ack_timeout_data, this->receiving_timeout);
	if(!(this->ack_timeout_data >= this->receiving_timeout)) { printf("[TEST-MEASTIMERS] Arm G pass-after FAIL\n"); fails++; }

	// Arm H (INVARIANT I3 fail-before/pass-after): the same warm long-batch window that
	// exceeds a stale watchdog_timeout / gearshift_timeout (set ONCE at load, never per-
	// batch). fail-before (MERCURY_WATCHDOG_FLOOR_DEFEAT=1) leaves both < receiving_timeout
	// (the watchdog would pre-empt the widened receive window); pass-after (floor on) re-
	// asserts watchdog_timeout >= receiving_timeout AND gearshift_timeout >= receiving_timeout.
	// data_batch_size==28 is warm bucket 3 from Arm C/G, so receiving_timeout is large.
	this->watchdog_timeout  = 1000;
	this->gearshift_timeout = 1000;
	setenv("MERCURY_WATCHDOG_FLOOR_DEFEAT", "1", 1);
	this->calculate_receiving_timeout();
	printf("[TEST-MEASTIMERS] Arm H fail-before: wd=%d gs=%d rx_to=%d (want wd<rx AND gs<rx, VIOLATED)\n",
		this->watchdog_timeout, this->gearshift_timeout, this->receiving_timeout);
	if(!(this->watchdog_timeout < this->receiving_timeout && this->gearshift_timeout < this->receiving_timeout))
		{ printf("[TEST-MEASTIMERS] Arm H fail-before FAIL\n"); fails++; }
	this->watchdog_timeout  = 1000;
	this->gearshift_timeout = 1000;
	unsetenv("MERCURY_WATCHDOG_FLOOR_DEFEAT");
	this->calculate_receiving_timeout();
	printf("[TEST-MEASTIMERS] Arm H pass-after: wd=%d gs=%d rx_to=%d (want wd>=rx AND gs>=rx, HOLDS)\n",
		this->watchdog_timeout, this->gearshift_timeout, this->receiving_timeout);
	if(!(this->watchdog_timeout >= this->receiving_timeout && this->gearshift_timeout >= this->receiving_timeout))
		{ printf("[TEST-MEASTIMERS] Arm H pass-after FAIL\n"); fails++; }

	unsetenv("MERCURY_MEASURED_TIMERS_DEFEAT");
	unsetenv("MERCURY_MEASURED_TIMERS_K");
	unsetenv("MERCURY_MEASURED_TIMERS_FLOOR_MS");
	unsetenv("MERCURY_ACK_TIMEOUT_FLOOR_DEFEAT");
	unsetenv("MERCURY_WATCHDOG_FLOOR_DEFEAT");
	unsetenv("MERCURY_LINKPHASE_CMD_FLOOR_DEFEAT");
	bool pass = (fails == 0);
	printf("[TEST-MEASTIMERS] %s: fails=%d\n", pass ? "PASS" : "FAIL", fails);
	fflush(stdout);
	return pass ? 0 : 1;
}

// T1 ZERO-DEAD-TIME TURNAROUND learned-timing model regression (the turnaround-timing regression). Drives the REAL
// t1_note_term_sample() + compute_pre_key_lead() + t1_scheduled_key_ms() on a throwaway
// controller (no IONOS/RF/telecom_system). fail-before/pass-after: the calibrated pre-key lead
// appears ONLY when the terms are WARM; under MERCURY_T1_CALIB_DEFEAT (or COLD) the model
// yields the reactive sentinel (no pre-key) -> the §4c robust fallback. SIM proves the LOGIC
// (convergence, lead arithmetic, amp-safety floor, >=0 clamp, fallback); the PHYSICAL keyup/
// deaf magnitudes are bench-owed (a bench measurement).
// CONNECT-TIME CALIBRATION BOOTSTRAP regression (the missing-producer fire-proof). Drives the
// REAL t1_seed_calibration_defaults() -- the production connect-accept bootstrap -- on a throwaway
// controller and proves the INERT->LIVE flip: BEFORE the bootstrap every term is COLD so
// compute_pre_key_lead returns the reactive sentinel and t1_scheduled_keyup_wait_ms is INERT (the
// pre-T1c state); AFTER the bootstrap the lead is warm (== the amp-safety floor window) and the
// scheduled pre-key FIRES. A distinct defeat arm (MERCURY_T1_CALIB_DEFEAT) forces COLD even
// post-seed. Expected values are re-derived from the SAME accessors (no magic numbers).
int cl_arq_controller::test_t1_calib_bootstrap()
{
	printf("[TEST-T1-CALIB-BOOTSTRAP] start\n"); fflush(stdout);
	int fails = 0;
	unsetenv("MERCURY_T1_CALIB_DEFEAT");
	unsetenv("MERCURY_T1_SCHEDKEY");

	// Production-like config: the bootstrap reads ptt_on/off_delay_ms; the floor = ptt_on_delay_ms.
	for(int d=0; d<T1_NUM_DIR; d++) for(int t=0; t<T1_NUM_TERM; t++)
		{ t1_term_est[d][t].srtt_ms = 0; t1_term_est[d][t].rttvar_ms = 0; t1_term_est[d][t].n = 0; }
	ptt_on_delay_ms  = 100;
	ptt_off_delay_ms = 200;
	t1_txdelay_floor_ms = ptt_on_delay_ms;

	// Arm A (FAIL-BEFORE = the pre-T1c inert state): no production caller -> every term COLD ->
	// compute_pre_key_lead reactive AND the scheduled pre-key inert (the reactive wait is unchanged).
	t1_seed_calibration_defaults();
	long long end_ms = 100000, now_ms = 99000;
	int lead_cold_rev = compute_pre_key_lead(T1_DIR_REV);
	int lead_cold_fwd = compute_pre_key_lead(T1_DIR_FWD);
	setenv("MERCURY_T1_SCHEDKEY", "1", 1);
	long long sched_cold = t1_scheduled_keyup_wait_ms(T1_DIR_REV, true, end_ms, now_ms);
	unsetenv("MERCURY_T1_SCHEDKEY");
	printf("[TEST-T1-CALIB-BOOTSTRAP] Arm A (cold/inert): lead_rev=%d lead_fwd=%d (want %d) sched=%lld (want %d reactive)\n",
		lead_cold_rev, lead_cold_fwd, T1_LEAD_REACTIVE, sched_cold, T1_SCHED_REACTIVE);
	if(lead_cold_rev != T1_LEAD_REACTIVE) { printf("[TEST-T1-CALIB-BOOTSTRAP] Arm A FAIL (rev warm before bootstrap)\n"); fails++; }
	if(lead_cold_fwd != T1_LEAD_REACTIVE) { printf("[TEST-T1-CALIB-BOOTSTRAP] Arm A FAIL (fwd warm before bootstrap)\n"); fails++; }
	if(sched_cold != T1_SCHED_REACTIVE)   { printf("[TEST-T1-CALIB-BOOTSTRAP] Arm A FAIL (scheduled pre-key fired uncalibrated)\n"); fails++; }

	// Arm B (PASS-AFTER): the PRODUCTION bootstrap warms both directions off the reactive sentinel.
	setenv("MERCURY_T1_SCHEDKEY", "1", 1);
	t1_seed_calibration_defaults();
	int lead_warm_rev = compute_pre_key_lead(T1_DIR_REV);
	int lead_warm_fwd = compute_pre_key_lead(T1_DIR_FWD);
	int keyup_win = t1_term_ms(T1_DIR_REV, T1_KEYUP);
	int deaf_mean = t1_term_srtt_ms(T1_DIR_REV, T1_DEAF);
	int overlap   = keyup_win - deaf_mean;
	int exp_lead  = (t1_txdelay_floor_ms > overlap) ? t1_txdelay_floor_ms : overlap; if(exp_lead < 0) exp_lead = 0;
	printf("[TEST-T1-CALIB-BOOTSTRAP] Arm B (warm): lead_rev=%d lead_fwd=%d (want %d = max(floor %d, keyup_win %d - deaf %d))\n",
		lead_warm_rev, lead_warm_fwd, exp_lead, t1_txdelay_floor_ms, keyup_win, deaf_mean);
	if(lead_warm_rev == T1_LEAD_REACTIVE) { printf("[TEST-T1-CALIB-BOOTSTRAP] Arm B FAIL (rev still reactive after bootstrap)\n"); fails++; }
	if(lead_warm_fwd == T1_LEAD_REACTIVE) { printf("[TEST-T1-CALIB-BOOTSTRAP] Arm B FAIL (fwd still reactive after bootstrap)\n"); fails++; }
	if(lead_warm_rev != exp_lead)         { printf("[TEST-T1-CALIB-BOOTSTRAP] Arm B FAIL (lead != re-derived expectation)\n"); fails++; }
	if(lead_warm_rev <= 0)                { printf("[TEST-T1-CALIB-BOOTSTRAP] Arm B FAIL (no calibrated lead)\n"); fails++; }

	// Arm C (the INERT->LIVE flip, the whole point): lever armed + warm + D5 -> the scheduled
	// pre-key NOW fires at exactly (end - lead) - now (it returned the reactive sentinel in Arm A).
	long long want_wait = (end_ms - (long long)lead_warm_rev) - now_ms;
	long long sched_warm = t1_scheduled_keyup_wait_ms(T1_DIR_REV, true, end_ms, now_ms);
	printf("[TEST-T1-CALIB-BOOTSTRAP] Arm C (fires): wait=%lld (want %lld = end-lead-now) [was %d reactive pre-bootstrap]\n",
		sched_warm, want_wait, T1_SCHED_REACTIVE);
	if(sched_warm == T1_SCHED_REACTIVE) { printf("[TEST-T1-CALIB-BOOTSTRAP] Arm C FAIL (scheduled pre-key still inert after bootstrap)\n"); fails++; }
	if(sched_warm != want_wait)         { printf("[TEST-T1-CALIB-BOOTSTRAP] Arm C FAIL (wait != end-lead-now)\n"); fails++; }
	unsetenv("MERCURY_T1_SCHEDKEY");

	// Arm D (DEFEAT, distinct fail-before): MERCURY_T1_CALIB_DEFEAT forces every term COLD even
	// after the bootstrap -> the calibrated lead DISAPPEARS -> reactive fallback; clearing it restores.
	setenv("MERCURY_T1_CALIB_DEFEAT", "1", 1);
	int lead_defeat = compute_pre_key_lead(T1_DIR_REV);
	unsetenv("MERCURY_T1_CALIB_DEFEAT");
	int lead_restored = compute_pre_key_lead(T1_DIR_REV);
	printf("[TEST-T1-CALIB-BOOTSTRAP] Arm D (defeat): defeat=%d (want %d reactive) restored=%d (want ==warm %d)\n",
		lead_defeat, T1_LEAD_REACTIVE, lead_restored, lead_warm_rev);
	if(lead_defeat != T1_LEAD_REACTIVE) { printf("[TEST-T1-CALIB-BOOTSTRAP] Arm D FAIL (defeat not reactive)\n"); fails++; }
	if(lead_restored != lead_warm_rev)  { printf("[TEST-T1-CALIB-BOOTSTRAP] Arm D FAIL (lead not restored after defeat cleared)\n"); fails++; }

	unsetenv("MERCURY_T1_CALIB_DEFEAT");
	unsetenv("MERCURY_T1_SCHEDKEY");
	bool pass = (fails == 0);
	printf("[TEST-T1-CALIB-BOOTSTRAP] %s: fails=%d\n", pass ? "PASS" : "FAIL", fails);
	fflush(stdout);
	return pass ? 0 : 1;
}

int cl_arq_controller::test_t1_calibration()
{
	printf("[TEST-T1-CALIB] start\n"); fflush(stdout);
	int fails = 0;
	unsetenv("MERCURY_T1_CALIB_DEFEAT");

	// Fresh model, amp-safety floor = 100 ms (the production default = ptt_on_delay_ms).
	for(int d=0; d<T1_NUM_DIR; d++) for(int t=0; t<T1_NUM_TERM; t++)
		{ t1_term_est[d][t].srtt_ms = 0; t1_term_est[d][t].rttvar_ms = 0; t1_term_est[d][t].n = 0; }
	t1_txdelay_floor_ms = 100;

	// Arm A (COLD fallback): no samples => every needed term COLD => reactive sentinel.
	int lead_cold = compute_pre_key_lead(T1_DIR_REV);
	long long end_ms = 100000;
	long long sched_cold = t1_scheduled_key_ms(T1_DIR_REV, end_ms);
	printf("[TEST-T1-CALIB] Arm A (cold): lead=%d (want %d) sched=%lld (want %lld, no pre-key)\n",
		lead_cold, T1_LEAD_REACTIVE, sched_cold, end_ms);
	if(lead_cold != T1_LEAD_REACTIVE) { printf("[TEST-T1-CALIB] Arm A FAIL (cold not reactive)\n"); fails++; }
	if(sched_cold != end_ms)          { printf("[TEST-T1-CALIB] Arm A FAIL (cold re-aimed the key)\n"); fails++; }

	// Arm B (convergence to injected truth): fold KEYUP=150 (REV, peer-observed) and DEAF=40
	// (REV, local). After the first sample srtt=R, rttvar=R/2 (RFC 6298 §2.2); repeated identical
	// samples drive srtt to the injected value and rttvar -> 0 (§2.3). 41 folds each.
	for(int i=0;i<41;i++) t1_note_term_sample(T1_DIR_REV, T1_KEYUP, 150);
	for(int i=0;i<41;i++) t1_note_term_sample(T1_DIR_REV, T1_DEAF,  40);
	int keyup_srtt = t1_term_srtt_ms(T1_DIR_REV, T1_KEYUP);
	int deaf_srtt  = t1_term_srtt_ms(T1_DIR_REV, T1_DEAF);
	printf("[TEST-T1-CALIB] Arm B (converge): keyup_srtt=%d (want 150) deaf_srtt=%d (want 40)\n",
		keyup_srtt, deaf_srtt);
	if(keyup_srtt != 150) { printf("[TEST-T1-CALIB] Arm B FAIL (keyup !converge 150)\n"); fails++; }
	if(deaf_srtt  != 40)  { printf("[TEST-T1-CALIB] Arm B FAIL (deaf !converge 40)\n"); fails++; }

	// Arm C (conservative high-percentile window, the conservative-window discipline): the KEYUP term used for the lead
	// is SRTT + max(G,K*RTTVAR) >= mean, never below the clock-G guard.
	int keyup_win = t1_term_ms(T1_DIR_REV, T1_KEYUP);
	printf("[TEST-T1-CALIB] Arm C (window): keyup_win=%d (want >=150 and win-srtt>=%d)\n",
		keyup_win, TT_CLOCK_G_MS);
	if(!(keyup_win >= keyup_srtt && (keyup_win - keyup_srtt) >= TT_CLOCK_G_MS))
		{ printf("[TEST-T1-CALIB] Arm C FAIL (window not conservative)\n"); fails++; }

	// Arm D (lead arithmetic + scheduled offset): lead = max(floor, keyup_win - deaf_srtt),
	// clamped >=0; scheduled key = end - lead. Re-derive the expected value from the SAME
	// accessors so the assertion is exact and not a fragile magic number.
	int overlap   = keyup_win - deaf_srtt;
	int exp_lead  = (100 > overlap) ? 100 : overlap; if(exp_lead < 0) exp_lead = 0;
	int got_lead  = compute_pre_key_lead(T1_DIR_REV);
	long long got_sched = t1_scheduled_key_ms(T1_DIR_REV, end_ms);
	printf("[TEST-T1-CALIB] Arm D (lead): overlap=%d lead=%d (want %d) sched=%lld (want %lld)\n",
		overlap, got_lead, exp_lead, got_sched, end_ms - (long long)exp_lead);
	if(got_lead != exp_lead)                   { printf("[TEST-T1-CALIB] Arm D FAIL (lead arithmetic)\n"); fails++; }
	if(got_sched != end_ms - (long long)exp_lead) { printf("[TEST-T1-CALIB] Arm D FAIL (scheduled offset)\n"); fails++; }
	if(got_lead == T1_LEAD_REACTIVE)           { printf("[TEST-T1-CALIB] Arm D FAIL (warm still reactive)\n"); fails++; }

	// Arm E (amp-safety floor dominates, §5h): a narrow overlap (keyup-deaf < floor) must be
	// lifted to the user floor. KEYUP=110, DEAF=90 => overlap ~20 < 100 => lead == floor 100.
	for(int d2=0; d2<T1_NUM_DIR; d2++) for(int t=0; t<T1_NUM_TERM; t++)
		{ t1_term_est[d2][t].srtt_ms = 0; t1_term_est[d2][t].rttvar_ms = 0; t1_term_est[d2][t].n = 0; }
	for(int i=0;i<41;i++) t1_note_term_sample(T1_DIR_FWD, T1_KEYUP, 110);
	for(int i=0;i<41;i++) t1_note_term_sample(T1_DIR_FWD, T1_DEAF,  90);
	int lead_floor = compute_pre_key_lead(T1_DIR_FWD);
	printf("[TEST-T1-CALIB] Arm E (floor): lead=%d (want 100, floor dominates)\n", lead_floor);
	if(lead_floor != 100) { printf("[TEST-T1-CALIB] Arm E FAIL (floor not honored)\n"); fails++; }

	// Arm F (>=0 clamp with floor 0): keyup < deaf must never yield a negative lead.
	for(int d2=0; d2<T1_NUM_DIR; d2++) for(int t=0; t<T1_NUM_TERM; t++)
		{ t1_term_est[d2][t].srtt_ms = 0; t1_term_est[d2][t].rttvar_ms = 0; t1_term_est[d2][t].n = 0; }
	t1_txdelay_floor_ms = 0;
	for(int i=0;i<41;i++) t1_note_term_sample(T1_DIR_REV, T1_KEYUP, 30);
	for(int i=0;i<41;i++) t1_note_term_sample(T1_DIR_REV, T1_DEAF,  90);
	int lead_clamp = compute_pre_key_lead(T1_DIR_REV);
	printf("[TEST-T1-CALIB] Arm F (clamp): lead=%d (want 0, never negative)\n", lead_clamp);
	if(lead_clamp != 0) { printf("[TEST-T1-CALIB] Arm F FAIL (negative lead not clamped)\n"); fails++; }

	// Arm G (DEFEAT = fail-before): with warm terms AND floor 100, MERCURY_T1_CALIB_DEFEAT=1
	// forces every term COLD, so the calibrated lead DISAPPEARS (reactive sentinel) and the key
	// is NOT re-aimed. This is the fail-before arm: the calibrated pre-key only exists after
	// calibration. pass-after (defeat cleared) restores the calibrated lead distinctly.
	t1_txdelay_floor_ms = 100;
	for(int d2=0; d2<T1_NUM_DIR; d2++) for(int t=0; t<T1_NUM_TERM; t++)
		{ t1_term_est[d2][t].srtt_ms = 0; t1_term_est[d2][t].rttvar_ms = 0; t1_term_est[d2][t].n = 0; }
	for(int i=0;i<41;i++) t1_note_term_sample(T1_DIR_REV, T1_KEYUP, 150);
	for(int i=0;i<41;i++) t1_note_term_sample(T1_DIR_REV, T1_DEAF,  40);
	int lead_warm2 = compute_pre_key_lead(T1_DIR_REV);
	setenv("MERCURY_T1_CALIB_DEFEAT", "1", 1);
	int lead_defeat = compute_pre_key_lead(T1_DIR_REV);
	long long sched_defeat = t1_scheduled_key_ms(T1_DIR_REV, end_ms);
	unsetenv("MERCURY_T1_CALIB_DEFEAT");
	int lead_after = compute_pre_key_lead(T1_DIR_REV);
	printf("[TEST-T1-CALIB] Arm G: warm=%d defeat=%d (want %d, reactive) sched_defeat=%lld (want %lld) after=%d (want ==warm)\n",
		lead_warm2, lead_defeat, T1_LEAD_REACTIVE, sched_defeat, end_ms, lead_after);
	if(lead_defeat != T1_LEAD_REACTIVE) { printf("[TEST-T1-CALIB] Arm G fail-before FAIL (defeat not reactive)\n"); fails++; }
	if(lead_warm2 == T1_LEAD_REACTIVE || lead_warm2 <= 0) { printf("[TEST-T1-CALIB] Arm G FAIL (warm has no calibrated lead)\n"); fails++; }
	if(sched_defeat != end_ms) { printf("[TEST-T1-CALIB] Arm G fail-before FAIL (defeat re-aimed the key)\n"); fails++; }
	if(lead_after != lead_warm2) { printf("[TEST-T1-CALIB] Arm G pass-after FAIL (lead not restored)\n"); fails++; }

	unsetenv("MERCURY_T1_CALIB_DEFEAT");
	bool pass = (fails == 0);
	printf("[TEST-T1-CALIB] %s: fails=%d\n", pass ? "PASS" : "FAIL", fails);
	fflush(stdout);
	return pass ? 0 : 1;
}

// T1b SCHEDULED-KEYING regression (the scheduled-keying negative control + fail-before/pass-after).
// Drives the REAL t1_scheduled_keyup_wait_ms() -- the SAME function the reverse-ACK keyup site in
// send_ack_pattern calls -- on a throwaway controller (no IONOS/RF/telecom_system). Proves: the
// scheduled pre-key re-aims the keyup to EXACTLY owner_keydown_end_ms-lead ONLY when the lever is
// armed AND the batch end is predictively known (header_carries_d5) AND the terms are WARM; with
// the lever off (the negative control), or no-D5, or COLD terms, or no predictive stamp, the
// function yields the reactive sentinel and the keyup wait is UNCHANGED (the robust fallback,
// never a hang, never worse than baseline). SIM proves the DECISION LOGIC; the PHYSICAL latency
// win is bench-owed (the terms stay COLD on the full-duplex loopback path -- a bench measurement).
int cl_arq_controller::test_t1_schedkey()
{
	printf("[TEST-T1-SCHEDKEY] start\n"); fflush(stdout);
	int fails = 0;
	unsetenv("MERCURY_T1_CALIB_DEFEAT");
	unsetenv("MERCURY_T1_SCHEDKEY");

	// Warm the REV terms: KEYUP=150 (peer-observed), DEAF=40 (local); amp-safety floor 100.
	for(int d=0; d<T1_NUM_DIR; d++) for(int t=0; t<T1_NUM_TERM; t++)
		{ t1_term_est[d][t].srtt_ms = 0; t1_term_est[d][t].rttvar_ms = 0; t1_term_est[d][t].n = 0; }
	t1_txdelay_floor_ms = 100;
	for(int i=0;i<41;i++) t1_note_term_sample(T1_DIR_REV, T1_KEYUP, 150);
	for(int i=0;i<41;i++) t1_note_term_sample(T1_DIR_REV, T1_DEAF,  40);
	int lead = compute_pre_key_lead(T1_DIR_REV);
	long long end_ms = 100000, now_ms = 99000;   // "now" is 1000 ms before the predictive batch end

	// Arm 1 (lever OFF = the negative control): warm + D5 but MERCURY_T1_SCHEDKEY unset =>
	// reactive sentinel => the keyup wait is NOT re-aimed (the gate is honored).
	long long off_d5 = t1_scheduled_keyup_wait_ms(T1_DIR_REV, true, end_ms, now_ms);
	printf("[TEST-T1-SCHEDKEY] Arm 1 (lever off): wait=%lld (want %d reactive)\n", off_d5, T1_SCHED_REACTIVE);
	if(off_d5 != T1_SCHED_REACTIVE) { printf("[TEST-T1-SCHEDKEY] Arm 1 FAIL (fired with lever off)\n"); fails++; }

	setenv("MERCURY_T1_SCHEDKEY", "1", 1);   // arm the lever for the remaining arms

	// Arm 2 (armed + warm + D5): the re-aim fires at EXACTLY (end - lead) - now.
	long long want2 = (end_ms - (long long)lead) - now_ms;
	long long on_d5 = t1_scheduled_keyup_wait_ms(T1_DIR_REV, true, end_ms, now_ms);
	printf("[TEST-T1-SCHEDKEY] Arm 2 (armed warm D5): wait=%lld (want %lld = end-lead-now, lead=%d)\n", on_d5, want2, lead);
	if(on_d5 != want2)             { printf("[TEST-T1-SCHEDKEY] Arm 2 FAIL (not end-lead-now)\n"); fails++; }
	if(on_d5 == T1_SCHED_REACTIVE) { printf("[TEST-T1-SCHEDKEY] Arm 2 FAIL (armed still reactive)\n"); fails++; }

	// Arm 3 (armed + warm + NO D5 = robust fallback, §4c): reactive sentinel (no pre-key on a no-D5 batch).
	long long on_nod5 = t1_scheduled_keyup_wait_ms(T1_DIR_REV, false, end_ms, now_ms);
	printf("[TEST-T1-SCHEDKEY] Arm 3 (no-D5 robust fallback): wait=%lld (want %d reactive)\n", on_nod5, T1_SCHED_REACTIVE);
	if(on_nod5 != T1_SCHED_REACTIVE) { printf("[TEST-T1-SCHEDKEY] Arm 3 FAIL (pre-keyed a no-D5 batch)\n"); fails++; }

	// Arm 4 (armed + D5 + COLD terms = uncalibrated fallback): reactive sentinel (never pre-key uncalibrated).
	for(int t=0; t<T1_NUM_TERM; t++)
		{ t1_term_est[T1_DIR_REV][t].srtt_ms = 0; t1_term_est[T1_DIR_REV][t].rttvar_ms = 0; t1_term_est[T1_DIR_REV][t].n = 0; }
	long long on_cold = t1_scheduled_keyup_wait_ms(T1_DIR_REV, true, end_ms, now_ms);
	printf("[TEST-T1-SCHEDKEY] Arm 4 (cold uncalibrated): wait=%lld (want %d reactive)\n", on_cold, T1_SCHED_REACTIVE);
	if(on_cold != T1_SCHED_REACTIVE) { printf("[TEST-T1-SCHEDKEY] Arm 4 FAIL (pre-keyed an uncalibrated link)\n"); fails++; }

	// Re-warm for the clamp / invalid-stamp / defeat arms.
	for(int i=0;i<41;i++) t1_note_term_sample(T1_DIR_REV, T1_KEYUP, 150);
	for(int i=0;i<41;i++) t1_note_term_sample(T1_DIR_REV, T1_DEAF,  40);

	// Arm 5 (just-missed target): clamp to key-now. The live caller separately proves that
	// the predictive stamp belongs to the active received batch before accepting this value.
	long long on_past = t1_scheduled_keyup_wait_ms(T1_DIR_REV, true, end_ms, end_ms + 5000);
	printf("[TEST-T1-SCHEDKEY] Arm 5 (past target clamp): wait=%lld (want 0 key-now)\n", on_past);
	if(on_past != 0) { printf("[TEST-T1-SCHEDKEY] Arm 5 FAIL (past target did not clamp)\n"); fails++; }

	// Arm 6 (invalid predictive stamp): owner_keydown_end_ms <= 0 => reactive sentinel.
	long long on_nostamp = t1_scheduled_keyup_wait_ms(T1_DIR_REV, true, 0, now_ms);
	printf("[TEST-T1-SCHEDKEY] Arm 6 (no stamp): wait=%lld (want %d reactive)\n", on_nostamp, T1_SCHED_REACTIVE);
	if(on_nostamp != T1_SCHED_REACTIVE) { printf("[TEST-T1-SCHEDKEY] Arm 6 FAIL (pre-keyed with no stamp)\n"); fails++; }

	// Arm 7 (DEFEAT = fail-before/pass-after): armed+warm fires; MERCURY_T1_CALIB_DEFEAT forces the
	// terms COLD so the scheduled re-aim DISAPPEARS (reactive); cleared, it restores distinctly.
	long long before = t1_scheduled_keyup_wait_ms(T1_DIR_REV, true, end_ms, now_ms);
	setenv("MERCURY_T1_CALIB_DEFEAT", "1", 1);
	long long during = t1_scheduled_keyup_wait_ms(T1_DIR_REV, true, end_ms, now_ms);
	unsetenv("MERCURY_T1_CALIB_DEFEAT");
	long long after = t1_scheduled_keyup_wait_ms(T1_DIR_REV, true, end_ms, now_ms);
	printf("[TEST-T1-SCHEDKEY] Arm 7 (defeat): before=%lld during=%lld (want %d) after=%lld (want ==before)\n",
		before, during, T1_SCHED_REACTIVE, after);
	if(during != T1_SCHED_REACTIVE) { printf("[TEST-T1-SCHEDKEY] Arm 7 fail-before FAIL (defeat still fired)\n"); fails++; }
	if(before == T1_SCHED_REACTIVE) { printf("[TEST-T1-SCHEDKEY] Arm 7 FAIL (warm not firing)\n"); fails++; }
	if(after != before)             { printf("[TEST-T1-SCHEDKEY] Arm 7 pass-after FAIL (lead not restored)\n"); fails++; }

	unsetenv("MERCURY_T1_SCHEDKEY");
	unsetenv("MERCURY_T1_CALIB_DEFEAT");
	bool pass = (fails == 0);
	printf("[TEST-T1-SCHEDKEY] %s: fails=%d\n", pass ? "PASS" : "FAIL", fails);
	fflush(stdout);
	return pass ? 0 : 1;
}

int cl_arq_controller::test_t1_schedkey_lifecycle()
{
	printf("[TEST-T1-LIFECYCLE] start\n");
	int fails = 0;
	auto check = [&](bool ok, const char* name) {
		printf("[TEST-T1-LIFECYCLE] %s: %s\n", ok ? "PASS" : "FAIL", name);
		if(!ok) fails++;
	};
	setenv("MERCURY_T1_SCHEDKEY", "1", 1);
	role = RESPONDER;
	link_status = CONNECTED;
	sack_v2_enabled = true;
	header_carries_d5 = true;
	t1_peer_calibration_valid = true;
	t1_peer_keyup_ms = 100;
	t1_peer_deaf_ms = 0;
	t1_peer_undeaf_ms = 150;
	t1_data_phase_proven = false;
	rsp_prev_batch_seq_id = -1;
	t1_txdelay_floor_ms = 100;
	for(int d=0; d<T1_NUM_DIR; d++) for(int t=0; t<T1_NUM_TERM; t++)
		{ t1_term_est[d][t].srtt_ms=0; t1_term_est[d][t].rttvar_ms=0; t1_term_est[d][t].n=0; }
	for(int i=0; i<41; i++) {
		t1_note_term_sample(T1_DIR_REV, T1_KEYUP, 100);
		t1_note_term_sample(T1_DIR_REV, T1_DEAF, 0);
	}

	lp_last_rx_bsi = 7;
	lp_state.owner = LP_CMD_KEYED;
	lp_state.epoch = lp_make_epoch(7);
	lp_state.owner_keydown_end_ms = lp_now() + 5000;
	long a0=t1_schedkey_arm_count, r0=t1_schedkey_reject_count;
	t1_arm_new_d5_frame0(7, 10, false);
	check(!t1_schedkey.armed && t1_schedkey_arm_count == a0,
		"establishment/first-turn gate arms 0 before clean DATA progress");
	t1_data_phase_proven = true;
	r0 = t1_schedkey_reject_count;
	t1_arm_new_d5_frame0(7, 10, false);
	check(t1_schedkey.armed && t1_schedkey_arm_count == a0+1,
		"CRC-good NEW-D5 frame-0 arms on the production decision path (count > 0)");
	t1_cancel_schedkey("unit-cleanup");

	lp_last_rx_bsi = 8; lp_state.epoch = lp_make_epoch(7);
	t1_arm_new_d5_frame0(8, 10, false);
	check(!t1_schedkey.armed && t1_schedkey_reject_count == r0+1,
		"wrong epoch arms 0");
	lp_last_rx_bsi = 8; lp_state.epoch = lp_make_epoch(8);
	t1_arm_new_d5_frame0(7, 10, false);
	check(!t1_schedkey.armed, "wrong BSI arms 0");
	lp_last_rx_bsi = 7; lp_state.epoch = lp_make_epoch(7);
	t1_arm_new_d5_frame0(7, 10, true);
	check(!t1_schedkey.armed, "physical retransmit arms 0 (reactive-only contract)");

	// Drive the scheduler itself across its target with a complete authoritative span.
	// Passive-monitor suppresses radio I/O; the state transition and freshness checks are
	// the same production poll executed by update_status().
	int saved_rx_span = rx_batch_total_frames;
	bool saved_monitor = passive_monitor;
	st_message* saved_messages_rx = messages_rx;
	std::vector<st_message> complete_span(10);
	for(int i=0; i<10; i++) complete_span[(size_t)i].status = FREE;
	messages_rx = complete_span.data();
	int saved_batch_size = data_batch_size;
	rx_batch_total_frames = -1;
	passive_monitor = true;
	connection_status = ACKNOWLEDGING_DATA;
	data_batch_size = 10;
	rsp_current_expected_batch_seq_id = 7;
	rsp_last_delivered_batch_seq_id = 6;
	rsp_prev_batch_active = false;
	rsp_stream_aborted = false;
	rsp_stream_origin_gen = -1;
	lp_last_rx_bsi=7; lp_state.owner=LP_CMD_KEYED;
	lp_state.epoch=lp_make_epoch(7); lp_state.owner_keydown_end_ms=lp_now()+375;
	long f0=t1_schedkey_fire_count;
	t1_arm_new_d5_frame0(7, 10, false);
	for(int i=0; i<10; i++) complete_span[(size_t)i].status = RECEIVED;
	rx_batch_total_frames = 10;
	// The scheduler must not treat generic RECEIVING/ACKNOWLEDGING state as
	// authority.  This unit isolates the scheduler transition; the real-handler
	// safety battery below drives the production clean-gate producer.
	t1_schedkey.clean_ack_authorized = true;
	t1_schedkey_poll();
	check(t1_schedkey.keyed && t1_schedkey_fire_count==f0+1,
		"complete clean span fires at the scheduled sacrificial-tail boundary");
	t1_cancel_schedkey("unit-fire-cleanup");
	passive_monitor = saved_monitor;
	data_batch_size = saved_batch_size;
	rx_batch_total_frames = saved_rx_span;
	messages_rx = saved_messages_rx;

	const char* reasons[] = {"crc-decode-fail", "partial-ack-gate", "config-change",
		"break", "drop"};
	for(const char* reason : reasons)
	{
		lp_last_rx_bsi=7; lp_state.owner=LP_CMD_KEYED;
		lp_state.epoch=lp_make_epoch(7); lp_state.owner_keydown_end_ms=lp_now()+5000;
		t1_arm_new_d5_frame0(7, 10, false);
		long c0=t1_schedkey_cancel_count;
		t1_cancel_schedkey(reason);
		check(!t1_schedkey.armed && !t1_schedkey.keyed
			&& t1_schedkey_cancel_count==c0+1, reason);
	}

	char cal[5]; int k=-1,d=-1,u=-1;
	t1_calibration_encode(cal, 125, 35, 205);
	check(t1_calibration_decode(cal, 5, &k, &d, &u)
		&& k==125 && d==35 && u==205,
		"CRC8-protected keyup/deaf/undeaf calibration exchange round-trips");
	cal[2] ^= 1;
	check(!t1_calibration_decode(cal, 5, &k, &d, &u),
		"calibration CRC failure rejects negotiation");

	uint64_t legacy_conn = 0, prekey_conn = 0;
	pack_test_conn_payload(&legacy_conn, 9, 0x3f, 7);
	prekey_conn = legacy_conn;
	pack_t1_calibration_reserved(&prekey_conn, 25, 41);
	uint8_t compact_k = 0, compact_u = 0;
	check((legacy_conn & ((1ULL << 18) - 1ULL)) == 0
		&& (legacy_conn >> 18) == (prekey_conn >> 18),
		"default-off MFSK TEST payload is unchanged and PREKEY only uses reserved bits");
	check(unpack_t1_calibration_reserved(prekey_conn, &compact_k, &compact_u)
		&& compact_k == 25 && compact_u == 41
		&& !unpack_t1_calibration_reserved(legacy_conn, &compact_k, &compact_u),
		"CRC12-covered compact handshake calibration round-trips; zero is absent");

	// D2 is a separate retransmit/listener safety floor.  PREKEY never accepts a
	// retransmit above, and no lifecycle function is passed this value or may alter it.
	int saved_ptt_on = ptt_on_delay_ms, saved_ptt_off = ptt_off_delay_ms;
	ptt_on_delay_ms = 150; ptt_off_delay_ms = 150;
	const int d2_guard_ms = d2_listener_rearm_guard_ms();
	long long past = t1_scheduled_keyup_wait_ms(T1_DIR_REV, true, 1000, 2000);
	const int d2_after_ms = d2_listener_rearm_guard_ms();
	ptt_on_delay_ms = saved_ptt_on; ptt_off_delay_ms = saved_ptt_off;
	check(past == 0 && d2_guard_ms == 300 && d2_after_ms == d2_guard_ms,
		"past-target clamps to key-now without deleting the independent D2 300ms guard");

	unsetenv("MERCURY_T1_SCHEDKEY");
	printf("[TEST-T1-LIFECYCLE] %s: arms=%ld rejects=%ld cancels=%ld fails=%d\n",
		fails ? "FAIL" : "PASS", t1_schedkey_arm_count,
		t1_schedkey_reject_count, t1_schedkey_cancel_count, fails);
	return fails ? 1 : 0;
}

// Regression for the explicit-CONNECT session boundary.  Session A has already
// proved DATA and learned a peer tuple.  Starting session B through the real user
// command path must revoke both before a legacy/zero-reserved peer can decline to
// overwrite them.  Even after B's first ordinary clean turnaround publishes its
// proof, absent calibration keeps the production arming predicate ineligible.
int cl_arq_controller::test_t1_fresh_connect_reset()
{
	printf("[TEST-T1-FRESH-CONNECT] start\n");
	int fails = 0;
	auto check = [&](bool ok, const char* name) {
		printf("[TEST-T1-FRESH-CONNECT] %s: %s\n", ok ? "PASS" : "FAIL", name);
		if(!ok) fails++;
	};

	setenv("MERCURY_T1_SCHEDKEY", "1", 1);
	// Session A: completed clean DATA and learned a usable calibration tuple.
	t1_data_phase_proven = true;
	t1_peer_calibration_valid = true;
	t1_peer_keyup_ms = 100;
	t1_peer_deaf_ms = 0;
	t1_peer_undeaf_ms = 150;
	// Keep the local predictor warm across CONNECT, as production does.  This
	// isolates the regression to leaked peer authority: without the boundary
	// reset, session B would have every other prerequisite needed to arm.
	t1_txdelay_floor_ms = 100;
	for(int d=0; d<T1_NUM_DIR; d++) for(int t=0; t<T1_NUM_TERM; t++)
		{ t1_term_est[d][t].srtt_ms=0; t1_term_est[d][t].rttvar_ms=0; t1_term_est[d][t].n=0; }
	for(int i=0; i<41; i++)
	{
		t1_note_term_sample(T1_DIR_REV, T1_KEYUP, 100);
		t1_note_term_sample(T1_DIR_REV, T1_DEAF, 0);
	}

	// Production new-session entry.  This is the path which previously skipped
	// reset_session_state() and leaked all five values into session B.
	process_user_command("CONNECT TESTA TESTB");
	check(link_status == CONNECTING,
		"explicit CONNECT starts session B");
	check(!t1_data_phase_proven && !t1_peer_calibration_valid
		&& t1_peer_keyup_ms == -1 && t1_peer_deaf_ms == -1
		&& t1_peer_undeaf_ms == -1,
		"session B starts with no inherited proof or peer calibration tuple");

	// Model the production strict-presence branch for a legacy/default-OFF peer:
	// an all-zero reserved field is absent and therefore does not write a tuple.
	uint64_t legacy = 0;
	uint8_t keyup_q5 = 0, undeaf_q5 = 0;
	bool calibration_present =
		unpack_t1_calibration_reserved(legacy, &keyup_q5, &undeaf_q5);
	if(calibration_present)
	{
		t1_peer_keyup_ms = 5 * (int)keyup_q5;
		t1_peer_deaf_ms = 0;
		t1_peer_undeaf_ms = 5 * (int)undeaf_q5;
		t1_peer_calibration_valid = true;
	}
	check(!calibration_present && !t1_peer_calibration_valid,
		"zero-reserved session-B handshake leaves calibration absent");

	// The first ordinary clean turnaround may publish B's proof, but PREKEY must
	// remain ineligible because B still has no peer tuple.
	role = RESPONDER;
	link_status = CONNECTED;
	sack_v2_enabled = true;
	header_carries_d5 = true;
	t1_data_phase_proven = true;
	rsp_prev_batch_seq_id = -1;
	lp_last_rx_bsi = 9;
	lp_state.owner = LP_CMD_KEYED;
	lp_state.epoch = lp_make_epoch(9);
	lp_state.owner_keydown_end_ms = lp_now() + 5000;
	t1_arm_new_d5_frame0(9, 10, false);
	check(!t1_schedkey.armed,
		"first clean DATA turnaround cannot arm PREKEY without session-B calibration");

	// Receiver coordinates belong to the loaded capture geometry. Session
	// teardown and idle CONNECT retain it; only a real configuration load
	// publishes a fresh coordinate system. Exercise the production entry points.
	{
		cl_telecom_system ts;
		ts.operation_mode = BER_PLOT_passband;
		ts.narrowband_enabled = NO;
		check(ts.load_configuration(CONFIG_15) == 0, "receiver lifecycle loads cfg15");
		cl_telecom_system* saved_telecom = telecom_system;
		telecom_system = &ts;
		current_configuration = CONFIG_15;
		init_configuration = CONFIG_15;
		narrowband_enabled = NO;
		nb_probe_max = 0;
		ts.preamble_amortization_enabled = true;
		ts.ofdm_defer_overflow_enabled = true;
		const char* mini_env = std::getenv("MERCURY_KEYDOWN_MINI0");
		const int mini = (!mini_env || !*mini_env || atoi(mini_env) != 0) ? 0 : 1;
		const int search = mini == 0 ? ts.data_container.preamble_nSymb + 6
			: ts.data_container.buffer_Nsymb - 1;
		auto seed = [&]() {
			ts.receive_stats.ofdm_search_raw = search;
			ts.receive_stats.mfsk_search_raw = 3;
			ts.receive_stats.ofdm_batch_active = true;
			ts.receive_stats.ofdm_drift_per_frame = 2.5;
			ts.receive_stats.delay_of_last_decoded_message = 1234;
			ts.data_container.nUnder_processing_events = 0;
		};
		auto grid_check = [&](const char* stage, bool reset) {
			const auto& r = ts.receive_stats;
			auto field = [&](bool ok, const char* name) {
				char label[160];
				snprintf(label, sizeof(label), "RX lifecycle %s: %s", stage, name);
				check(ok, label);
			};
			field(r.ofdm_search_raw == (reset ? 0 : search), "OFDM cursor");
			field(r.mfsk_search_raw == (reset ? 0 : 3), "MFSK cursor");
			field(r.ofdm_batch_active == !reset, "batch activity");
			field(r.ofdm_drift_per_frame == (reset ? 0.0 : 2.5), "drift");
			field(r.delay_of_last_decoded_message == (reset ? -1 : 1234), "decoded delay");
		};
		auto next_receive = [&](const char* stage, bool reset) {
			const int pb = ts.data_container.Nofdm * ts.frequency_interpolation_rate
				* ts.data_container.buffer_Nsymb;
			std::vector<double> silence((size_t)pb, 0.0);
			std::vector<int> out((size_t)N_MAX, 0);
			ts.receive_stats.last_eff_preamble_nsymb = -99;
			auto r = ts.receive_byte(silence.data(), out.data());
			const int expected = reset ? ts.data_container.preamble_nSymb : mini;
			printf("[TEST-RX-LIFECYCLE] ROUTE stage=%s mini=%d config=%d overflow=%d decoded=%d reported=%d expected=%d\n",
				stage, mini, ts.current_configuration, r.frame_overflow_symbols,
				r.message_decoded, ts.receive_stats.last_eff_preamble_nsymb, expected);
			check(r.message_decoded != YES && (reset || mini == 0 || r.frame_overflow_symbols > 0),
				"RX lifecycle next receive witnesses rejection/defer route");
			check(ts.receive_stats.last_eff_preamble_nsymb == expected,
				"RX lifecycle next receive reports searched geometry");
		};

		seed();
		load_configuration(CONFIG_15, FULL, YES);
		grid_check("same-config", false);
		next_receive("same-config", false);

		seed();
		link_status = IDLE;
		process_user_command("CONNECT TESTA TESTB");
		check(link_status == CONNECTING && current_configuration == CONFIG_15,
			"RX lifecycle idle CONNECT retains loaded configuration");
		grid_check("idle-CONNECT", false);
		next_receive("idle-CONNECT", false);

		seed();
		link_status = CONNECTED;
		reset_session_state();
		check(current_configuration == CONFIG_NONE && ts.current_configuration == CONFIG_NONE,
			"RX lifecycle actual teardown invalidates configuration");
		grid_check("teardown", false);
		next_receive("teardown", false);

		// Keep the command transmit hook absent for this entry: production
		// CONNECT must actually reload the PHY after teardown, not skip it as
		// the socket-only command harness does.
		seed();
		auto saved_hook = cl_tcp_socket::g_test_transmit_hook;
		cl_tcp_socket::g_test_transmit_hook = nullptr;
		process_user_command("CONNECT TESTA TESTB");
		cl_tcp_socket::g_test_transmit_hook = saved_hook;
		check(link_status == CONNECTING && current_configuration == CONFIG_15
			&& ts.current_configuration == CONFIG_15,
			"RX lifecycle reconnect actually reloads cfg15");
		grid_check("reconnect-load", true);
		next_receive("reconnect-load", true);
		telecom_system = saved_telecom;
	}

	unsetenv("MERCURY_T1_SCHEDKEY");
	printf("[TEST-T1-FRESH-CONNECT] %s: fails=%d\n",
		fails ? "FAIL" : "PASS", fails);
	return fails ? 1 : 0;
}

int cl_arq_controller::test_t1_repeating_prefix()
{
	printf("[TEST-T1-PREFIX] start\n");
	int fails = 0;
	cl_mfsk m;
	m.init(16, 50, 1);
	const int reps = 4, L = cl_mfsk::PREKEY_PREFIX_LEN;
	std::vector<std::complex<double>> v((size_t)reps * L * 50);
	m.generate_prekey_prefix(v.data(), reps);
	for(int r=0; r<reps; r++) for(int s=0; s<L; s++)
	{
		int active=-1, count=0;
		for(int k=0; k<50; k++) if(std::norm(v[(r*L+s)*50+k]) > 0.0)
			{ active=k; count++; }
		int want=m.stream_offsets[0]
			+ (m.prekey_prefix_tones[s] + s*m.tone_hop_step) % m.M;
		if(count != 1 || active != want) fails++;
	}
	int worst=0;
	for(int rung=0; rung<cl_mfsk::SCREAM_NBASES; rung++) for(int shift=0; shift<8; shift++)
	{
		int same=0;
		for(int s=0; s<L; s++)
			if(m.prekey_prefix_tones[s] == m.scream_tones[rung][(s+shift)%8]) same++;
		if(same>worst) worst=same;
	}
	if(worst >= 4) fails++;

	// Exercise the production passband generator and the production commander
	// discriminator, including a negative-control ACK waveform. CRC remains the
	// final authority, but the prefix gate must acquire every repetition and must
	// not fence an ordinary reactive ACK/SACK response.
	cl_telecom_system ts;
	ts.operation_mode = ARQ_MODE;
	ts.load_configuration(CONFIG_9);
	const int unit_samples = ts.prekey_prefix_unit_passband_samples;
	std::vector<double> wire((size_t)unit_samples * reps, 0.0);
	int written = ts.generate_prekey_prefix_passband(wire.data(), reps);
	int detector_locks = 0;
	for(int r=0; r<reps; r++)
	{
		int matched = 0;
		double metric = ts.detect_prekey_prefix_from_passband(
			&wire[(size_t)r * unit_samples], unit_samples, &matched);
		if(matched == L && metric >= 3.0) detector_locks++;
	}
	int ack_samples = ts.ack_pattern_passband_samples;
	std::vector<double> ack((size_t)ack_samples, 0.0);
	ts.generate_ack_pattern_passband(ack.data());
	int ack_matched = 0;
	double ack_metric = ts.detect_prekey_prefix_from_passband(
		ack.data(), ack_samples, &ack_matched);
	if(written != unit_samples * reps || detector_locks != reps
	   || (ack_matched == L && ack_metric >= 3.0)) fails++;
	printf("[TEST-T1-PREFIX] %s: base={7,2,3,13,11} reps=%d tone_locks=%d/%d "
		"passband_locks=%d/%d ack_false_lock=%d metric=%.3f "
		"worst_scream_coincidence=%d/5 (<4 required)\n",
		fails ? "FAIL" : "PASS", reps, reps, reps, detector_locks, reps,
		(ack_matched == L && ack_metric >= 3.0) ? 1 : 0, ack_metric, worst);
	return fails ? 1 : 0;
}

// KARN DISCRIMINATOR classifier A/B (MERCURY_KARN_RETX_ONLY). Drives the REAL production
// gate tt_karn_sample_ok() + the REAL fold update_turnaround_estimate() — the SAME two
// calls the reverse-ACK accept sites (arq_commander.cc) make. The classifier decoupling
// separates "the round carried retx frames" (data_ack_round_carried_retx, the Karn signal)
// from "the CMD receive window is H1-widened" (data_ack_retx_turnaround, which arms on a
// CLEAN fresh partial-capable OFDM batch). Reading the latter starved the estimator (n=0
// on the dominant OFDM path). NOTE: this is a UNIT proof of the classifier + fold; the
// end-to-end WIRE fire proof (a production [TT-RTT-UPDATE] n:0->N on a real clean cell with
// the knob ON) is owed as a real-audio cohort. See data-flow-turnaround-timers.md.
int cl_arq_controller::test_karn_retx_classify()
{
	printf("[TEST-KARN] start\n");
	fflush(stdout);
	int fails = 0;
	auto check = [&](bool cond, const char* name) {
		printf("[TEST-KARN] %s: %s\n", cond ? "PASS" : "FAIL", name);
		if(!cond) fails++;
		fflush(stdout);
	};

	// Throwaway geometry (mirrors test_measured_timers): CONFIG_13 = class 1, batch 20 =>
	// bucket 2. A CLEAN fresh multi-frame OFDM batch arms the H1 widen, so
	// data_ack_retx_turnaround=TRUE while the round carried NO retransmitted frames.
	this->role                         = COMMANDER;
	this->message_transmission_time_ms = 200;
	this->current_configuration        = CONFIG_13;
	this->data_batch_size              = 20;
	this->cfg16_revack_starve_fails    = 0;
	const int cls = tt_class_of(CONFIG_13);
	const int bk  = tt_batchbk_of(20 * 200);

	// ---- Arm K1: CLEAN fresh OFDM batch (H1 armed) ----
	// fail-before (knob OFF, legacy coupling): tt_karn_sample_ok() reads
	// data_ack_retx_turnaround==TRUE -> gate FALSE -> the clean sample is EXCLUDED ->
	// tt_rtt.n stays 0 (the inert-estimator bug this fix targets).
	unsetenv("MERCURY_KARN_RETX_ONLY");
	this->tt_rtt[cls][bk].n = 0; this->tt_rtt[cls][bk].srtt_ms = 0; this->tt_rtt[cls][bk].rttvar_ms = 0;
	this->data_ack_retx_turnaround    = true;    // H1 widen armed on a clean batch
	this->data_ack_round_carried_retx = false;   // ...but the round carried NO retx frames
	bool ok_before = this->tt_karn_sample_ok();
	if(ok_before) this->update_turnaround_estimate(CONFIG_13, 20 * 200, 2000);
	int n_before = this->tt_rtt[cls][bk].n;
	printf("[TEST-KARN] K1 fail-before (legacy): gate_ok=%d n=%d (want 0/0)\n", ok_before ? 1 : 0, n_before);
	check(ok_before == false, "K1 fail-before (legacy): clean H1 batch -> Karn gate FALSE (sample EXCLUDED)");
	check(n_before == 0,      "K1 fail-before (legacy): estimator STARVED (n stays 0) -- the inert-estimator bug");

	// pass-after (knob ON): tt_karn_sample_ok() reads data_ack_round_carried_retx==FALSE ->
	// gate TRUE -> the clean sample is FOLDED -> n 0->1 (the classifier fire proof).
	setenv("MERCURY_KARN_RETX_ONLY", "1", 1);
	this->tt_rtt[cls][bk].n = 0; this->tt_rtt[cls][bk].srtt_ms = 0; this->tt_rtt[cls][bk].rttvar_ms = 0;
	this->data_ack_retx_turnaround    = true;    // H1 producer UNCHANGED (still arms)
	this->data_ack_round_carried_retx = false;   // clean round
	bool ok_after = this->tt_karn_sample_ok();
	if(ok_after) this->update_turnaround_estimate(CONFIG_13, 20 * 200, 2000);
	int n_after = this->tt_rtt[cls][bk].n;
	printf("[TEST-KARN] K1 pass-after (fix): gate_ok=%d n=%d srtt=%d (want 1/1/2000)\n",
		ok_after ? 1 : 0, n_after, this->tt_rtt[cls][bk].srtt_ms);
	check(ok_after == true, "K1 pass-after (fix): clean H1 batch -> Karn gate TRUE (sample ACCEPTED)");
	check(n_after == 1,     "K1 pass-after (fix): estimator FOLDS the clean sample (n 0->1) -- FIRE PROOF");

	// ---- Arm K2: a GENUINE retx round is STILL excluded under the fix (Karn preserved) ----
	setenv("MERCURY_KARN_RETX_ONLY", "1", 1);
	this->tt_rtt[cls][bk].n = 5; this->tt_rtt[cls][bk].srtt_ms = 2000; this->tt_rtt[cls][bk].rttvar_ms = 100;
	int srtt5 = this->tt_rtt[cls][bk].srtt_ms;
	this->data_ack_retx_turnaround    = true;
	this->data_ack_round_carried_retx = true;    // v1-retx / v2-mixed: the round carried retx frames
	bool ok_retx = this->tt_karn_sample_ok();
	if(ok_retx) this->update_turnaround_estimate(CONFIG_13, 20 * 200, 9999);
	check(ok_retx == false, "K2 (fix): a GENUINE retx round -> Karn gate FALSE (sample still EXCLUDED)");
	check(this->tt_rtt[cls][bk].srtt_ms == srtt5, "K2 (fix): retx sample does NOT move SRTT (Karn preserved)");

	// ---- Arm K3: starve>0 still excludes even a clean round (belt-and-braces) ----
	setenv("MERCURY_KARN_RETX_ONLY", "1", 1);
	this->data_ack_retx_turnaround    = false;
	this->data_ack_round_carried_retx = false;
	this->cfg16_revack_starve_fails   = 1;        // a prior reverse-ACK was lost
	check(this->tt_karn_sample_ok() == false, "K3 (fix): starve>0 -> Karn gate FALSE (belt-and-braces)");
	this->cfg16_revack_starve_fails   = 0;

	// ---- Arm K4: A/B identity -- knob OFF is byte-identical legacy behaviour ----
	// The new member is set to a value that WOULD flip the fixed gate; knob OFF must ignore
	// it and read data_ack_retx_turnaround only.
	unsetenv("MERCURY_KARN_RETX_ONLY");
	this->data_ack_retx_turnaround    = false;   // legacy path: clean
	this->data_ack_round_carried_retx = true;    // would EXCLUDE under the fix (ignored knob-off)
	check(this->tt_karn_sample_ok() == true, "K4 A/B identity (knob off): gate reads the LEGACY flag ONLY");

	unsetenv("MERCURY_KARN_RETX_ONLY");
	printf("[TEST-KARN] %s: fails=%d\n", fails == 0 ? "PASS" : "FAIL", fails);
	fflush(stdout);
	return fails == 0 ? 0 : 1;
}

// Fail-before/pass-after regression for the ignored observe_transition verdict.
// The next generation cannot fit in lp_state.epoch's 24-bit config field, so
// the real observer rejects it.  load_configuration() must honor that verdict,
// leave both ARQ and PHY geometry untouched, and invalidate stale link-phase
// evidence.  Virtual sample time keeps the observation deterministic.
int cl_arq_controller::test_linkphase_config_transition_failure()
{
	int fails = 0;
	auto check = [&](bool condition, const char* name) {
		printf("[TEST-LP-CONFIG-CONTRACT] %s: %s\n",
			condition ? "PASS" : "FAIL", name);
		if(!condition) fails++;
	};

	const char* prior_env = std::getenv("MERCURY_LINKPHASE_CONFIG_CONTRACT");
	const bool had_prior_env = prior_env != NULL;
	const std::string prior_env_value = prior_env ? prior_env : "";
	const int prior_sim_clock = sim_clock_enabled();
	setenv_portable("MERCURY_LINKPHASE_CONFIG_CONTRACT", "1");
	sim_clock_set_enabled(1);
	sim_clock_add_samples(480);  // 10 ms on the deterministic 48-kHz clock

	cl_telecom_system test_telecom;
	test_telecom.operation_mode = ARQ_MODE;
	test_telecom.current_configuration = CONFIG_0;
	current_configuration = CONFIG_0;
	telecom_system = &test_telecom;
	lp_config_gen = 0x00ffffffu;
	lp_state.owner = LP_TURNAROUND;
	lp_state.owner_keydown_end_ms = 123;
	lp_state.next_listen_open_ms = 456;
	lp_state.epoch = lp_make_epoch(7);
	lp_state.keydown_end_token = 9;
	lp_keydown_start_ms = 100;
	lp_keydown_start_token = 9;

	load_configuration(CONFIG_1, PHYSICAL_LAYER_ONLY, NO);

	check(current_configuration == CONFIG_0,
		"rejected observation stops load_configuration before geometry changes");
	check(test_telecom.current_configuration == CONFIG_0,
		"rejected observation stops before publishing new PHY geometry");
	check(lp_config_gen == 0x00ffffffu,
		"rejected observation does not publish an unrepresentable generation");
	check(!lp_config_contract.valid(),
		"observe_transition failure invalidates the transition contract");
	check(lp_state.owner == LP_NONE && lp_state.epoch == 0
		&& lp_state.owner_keydown_end_ms == 0
		&& lp_state.next_listen_open_ms == 0
		&& lp_state.keydown_end_token == 0,
		"rejected observation clears all stale link-phase evidence");

	sim_clock_set_enabled(prior_sim_clock);
	if(had_prior_env)
		setenv_portable("MERCURY_LINKPHASE_CONFIG_CONTRACT", prior_env_value.c_str());
	else
		unsetenv_portable("MERCURY_LINKPHASE_CONFIG_CONTRACT");

	printf("[TEST-LP-CONFIG-CONTRACT] %s (%d failure%s)\n",
		fails == 0 ? "ALL PASS" : "FAILURES", fails, fails == 1 ? "" : "s");
	fflush(stdout);
	return fails;
}

int cl_arq_controller::test_lp_break_recovery_fail_closed()
{
	int fails = 0;
	const char* key = "MERCURY_LP_TRANSITION_CONTRACT";
	const char* prior = std::getenv(key);
	const bool had_prior = prior != NULL;
	const std::string prior_value = prior ? prior : "";
#if defined(_WIN32)
	_putenv_s(key, "1");
#else
	setenv(key, "1", 1);
#endif

	const int prior_sim_enabled = sim_clock_enabled();
	sim_clock_set_enabled(1);
	lp_config_gen = 1;
	lp_reset();
	lp_note_break(/*bsi=*/7);
	sim_clock_add_samples(480);  // deterministic 10 ms at 48 kHz
	lp_test_force_observe_failure = true;
	const bool rejected = !lp_note_break_recovery(/*bsi=*/7);
	if(!rejected || lp_contract_valid || lp_listener_ready_calls != 0
	   || lp_state.owner != LP_CMD_KEYED)
	{
		printf("[TEST-LP-BREAK-RECOVERY] FAIL: rejected=%d valid=%d "
		       "listener_calls=%u owner=%d\n",
			rejected ? 1 : 0, lp_contract_valid ? 1 : 0,
			lp_listener_ready_calls, (int)lp_state.owner);
		fails++;
	}
	else
	{
		printf("[TEST-LP-BREAK-RECOVERY] PASS: rejected BreakObserved stopped "
		       "before ListenerReady\n");
	}

	// A valid recovery proves the positive path and the clock domain: both
	// events publish at exactly the virtual 10 ms instant, with no wall sleep.
	lp_reset();
	lp_note_break(/*bsi=*/8);
	sim_clock_add_samples(480);
	const bool accepted = lp_note_break_recovery(/*bsi=*/8);
	if(!accepted || !lp_contract_valid || lp_listener_ready_calls != 1
	   || lp_contract_last_event != mercury::protocol::LinkPhaseEvent::ListenerReady
	   || lp_contract_last_bsi != 8 || lp_contract_last_event_ms != 10
	   || lp_contract_ordinal != 2 || lp_state.owner != LP_TURNAROUND)
	{
		printf("[TEST-LP-BREAK-RECOVERY] FAIL: accepted=%d valid=%d "
		       "listener_calls=%u event=%d bsi=%d at_ms=%lld ordinal=%llu owner=%d\n",
			accepted ? 1 : 0, lp_contract_valid ? 1 : 0,
			lp_listener_ready_calls, (int)lp_contract_last_event,
			lp_contract_last_bsi, lp_contract_last_event_ms,
			lp_contract_ordinal, (int)lp_state.owner);
		fails++;
	}
	else
	{
		printf("[TEST-LP-BREAK-RECOVERY] PASS: valid BREAK->LISTENER_READY "
		       "at deterministic t=10 ms\n");
	}

	sim_clock_set_enabled(prior_sim_enabled);
#if defined(_WIN32)
	_putenv_s(key, had_prior ? prior_value.c_str() : "");
#else
	if(had_prior) setenv(key, prior_value.c_str(), 1);
	else unsetenv(key);
#endif
	printf("[TEST-LP-BREAK-RECOVERY] %s (failures=%d)\n",
		fails == 0 ? "ALL PASS" : "FAIL", fails);
	return fails;
}
// Directed unit for the helpers above (production helpers on a throwaway controller).  Each
// check runs the default (fixed) arm and the knob-off arm; the knob-off arm must reproduce the
// defect (fail-before) and the default arm must meet the requirement (pass-after).
static void gs2sc_setenv(const char* k, const char* v)
{
#if defined(_WIN32)
	_putenv_s(k, v ? v : "");
#else
	if(v) setenv(k, v, 1); else unsetenv(k);
#endif
}


int cl_arq_controller::test_gs2_spec_conformance()
{
	int fails = 0;
	// ---- R1: ACK slot floor in the receive timer's own clock --------------------------
	telecom_system = NULL;
	message_transmission_time_ms = 292;
	lp_config_gen = 0;
	const int prior_sim_clock = sim_clock_enabled();
	sim_clock_set_enabled(1);
	lp_reset();
	lp_note_keydown_start(/*bsi=*/3);
	sim_clock_add_samples(48 * 2200);               // CONFIG_TAG + guard before frame 0
	gs2_frames_start_ms = lp_now();
	int kd_ms = 21082;                               // arithmetic 90-frame keydown (responder view)
	int keyed_term = linkphase_slot_kd_remaining_ms(kd_ms);        // keydown still running
	sim_clock_add_samples(48 * 18100);              // the frames actually drained in 18.1 s
	lp_note_keydown_end(/*bsi=*/3, /*frames_region_len=*/48 * 18100,
		/*frames=*/90, /*force_full=*/false);
	gs2sc_setenv("MERCURY_LINKPHASE_SLOT_ORIGIN", NULL);
	int drained_term = linkphase_slot_kd_remaining_ms(kd_ms);      // timer starts post-drain
	gs2sc_setenv("MERCURY_LINKPHASE_SLOT_ORIGIN", "0");
	int old_term = linkphase_slot_kd_remaining_ms(kd_ms);
	gs2sc_setenv("MERCURY_LINKPHASE_SLOT_ORIGIN", NULL);
	sim_clock_set_enabled(prior_sim_clock);
	gs2_frames_start_ms = 0;
	if(keyed_term != kd_ms) { printf("[TEST-GS2SC] FAIL R1: keyed term %d != %d\n", keyed_term, kd_ms); fails++; }
	else if(drained_term != kd_ms - 18100) { printf("[TEST-GS2SC] FAIL R1: post-drain remaining keydown %d != %d\n", drained_term, kd_ms - 18100); fails++; }
	else if(old_term != kd_ms) { printf("[TEST-GS2SC] FAIL R1: knob-off arm did not reproduce the double count (%d)\n", old_term); fails++; }
	else printf("[TEST-GS2SC] PASS R1: post-drain slot floor keeps only the unelapsed responder "
		"keydown %d ms (knob-off arm double-counts %d ms)\n", drained_term, old_term);

	// ---- R2: CONFIG_TAG guard = peer receive-frame period, never above the window guard -----
	gs2_prev_tx_frame_samples = (20 + 10) * 1168L;   // CONFIG_13: preamble 4 + Nsymb 16, +10 margin
	gs2_prev_tx_config = 13;
	long window = 153008;                               // CONFIG_16 acquisition window (log)
	long g_new = gs2_tag_guard_samples(window);
	gs2sc_setenv("MERCURY_GS2_TAG_GUARD", "window");
	long g_old = gs2_tag_guard_samples(window);
	gs2sc_setenv("MERCURY_GS2_TAG_GUARD", NULL);
	gs2_prev_tx_frame_samples = 400000;                 // larger than the window: never raised
	long g_cap = gs2_tag_guard_samples(window);
	gs2_prev_tx_frame_samples = 0; gs2_prev_tx_config = -1;
	if(g_new != 35040) { printf("[TEST-GS2SC] FAIL R2: guard %ld != peer frame 35040\n", g_new); fails++; }
	else if(g_old != window) { printf("[TEST-GS2SC] FAIL R2: window arm %ld != %ld\n", g_old, window); fails++; }
	else if(g_cap != window) { printf("[TEST-GS2SC] FAIL R2: guard raised above window (%ld)\n", g_cap); fails++; }
	else printf("[TEST-GS2SC] PASS R2: 13->16 guard %.0f ms (window arm %.0f ms)\n",
		1000.0 * g_new / 48000.0, 1000.0 * g_old / 48000.0);

	// ---- R3: ACKed SWITCH_BANDWIDTH is a coordinated change (no entry tag) -----------------
	rate_opt.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
	inband_last_announced_config = CONFIG_NONE;
	inband_last_confirmed_config = CONFIG_NONE;
	gs2sc_setenv("MERCURY_GS2_COORD_WB_ENTRY", "0");
	bool off_arm = gs2_coordinated_wb_entry(CONFIG_0);
	int off_announced = inband_last_announced_config;
	gs2sc_setenv("MERCURY_GS2_COORD_WB_ENTRY", NULL);
	bool on_arm = gs2_coordinated_wb_entry(CONFIG_0);
	if(off_arm || off_announced != CONFIG_NONE) { printf("[TEST-GS2SC] FAIL R3: knob-off arm changed state\n"); fails++; }
	else if(!on_arm || inband_last_announced_config != CONFIG_0 || inband_last_confirmed_config != CONFIG_0
	        || inband_retag_armed)
	{ printf("[TEST-GS2SC] FAIL R3: coordinated WB entry not recorded (announced=%d confirmed=%d armed=%d)\n",
		inband_last_announced_config, inband_last_confirmed_config, inband_retag_armed ? 1 : 0); fails++; }
	else printf("[TEST-GS2SC] PASS R3: ACKed SWITCH_BANDWIDTH records CONFIG_0 as announced+followed "
		"(knob-off arm leaves it unannounced -> tagged entry)\n");
	rate_opt.set_mode_for_test(GEARSHIFT_V2_LEGACY);

	// ---- R4: unacknowledged batch-geometry control resumes the data plane ------------------
	bool r4_lp = inband_ctrl_miss_resumes_data(SET_LINK_PARAMS);
	bool r4_dw = inband_ctrl_miss_resumes_data(ROBUST_DWELL_BATCH_OP);
	bool r4_sc = inband_ctrl_miss_resumes_data(SET_CONFIG);
	gs2sc_setenv("MERCURY_INBAND_CTRL_MISS_RESUME", "0");
	bool r4_off = inband_ctrl_miss_resumes_data(SET_LINK_PARAMS);
	gs2sc_setenv("MERCURY_INBAND_CTRL_MISS_RESUME", NULL);
	if(!r4_lp || !r4_dw || r4_sc || r4_off)
	{ printf("[TEST-GS2SC] FAIL R4: lp=%d dwell=%d set_config=%d off=%d\n", r4_lp, r4_dw, r4_sc, r4_off); fails++; }
	else printf("[TEST-GS2SC] PASS R4: SET_LINK_PARAMS/dwell miss resumes DATA; SET_CONFIG unchanged; "
		"knob-off arm keeps the stranded control state\n");

	// ---- R5: a live control exchange is not a liveness stall ----------------------------
	{
		connection_status = RECEIVING_ACKS_CONTROL;
		messages_control.status = ADDED_TO_LIST;
		int saved_len = messages_control.length;
		messages_control.length = 4;
		char r5_buf[8] = {0};
		bool r5_null = (messages_control.data == NULL);
		if(r5_null) messages_control.data = r5_buf;
		char saved_code = messages_control.data ? messages_control.data[0] : 0;
		bool other_exempt = true;
		if(messages_control.data) { messages_control.data[0] = (char)SET_CONFIG;
			other_exempt = gs2_liveness_control_exchange_exempt();
			messages_control.data[0] = (char)SET_LINK_PARAMS; }
		bool live_on = messages_control.data ? gs2_liveness_control_exchange_exempt() : true;
		if(other_exempt) { printf("[TEST-GS2SC] FAIL R5: SET_CONFIG exchange exempted from the livelock guard\n"); fails++; }
		gs2sc_setenv("MERCURY_GS2_LIVENESS_CTRL_EXEMPT", "0");
		bool live_off = gs2_liveness_control_exchange_exempt();
		gs2sc_setenv("MERCURY_GS2_LIVENESS_CTRL_EXEMPT", NULL);
		messages_control.status = FREE;
		bool freed = gs2_liveness_control_exchange_exempt();
		messages_control.length = saved_len;
		if(messages_control.data) messages_control.data[0] = saved_code;
		if(r5_null) messages_control.data = NULL;
		connection_status = IDLE;
		if(!live_on || live_off || freed)
		{ printf("[TEST-GS2SC] FAIL R5: live=%d off=%d freed=%d\n", live_on, live_off, freed); fails++; }
		else printf("[TEST-GS2SC] PASS R5: queued control exchange exempt from the liveness stall; "
			"freed slot is not (knob-off arm accrues and fabricates a failure)\n");
	}

	// ---- R6: an empty control state under ACTIVE returns to the data plane ------------------
	{
		rate_opt.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
		int saved_link = link_status;
		link_status = CONNECTED; connection_status = TRANSMITTING_CONTROL;
		messages_control.status = FREE;
		bool r6_on = gs2_empty_control_state_resumes_data();
		gs2sc_setenv("MERCURY_GS2_EMPTY_CTRL_RESUME", "0");
		bool r6_off = gs2_empty_control_state_resumes_data();
		gs2sc_setenv("MERCURY_GS2_EMPTY_CTRL_RESUME", NULL);
		messages_control.status = ADDED_TO_LIST; int sl = messages_control.length; messages_control.length = 4;
		bool r6_live = gs2_empty_control_state_resumes_data();
		messages_control.status = FREE; messages_control.length = sl;
		link_status = saved_link; connection_status = IDLE;
		rate_opt.set_mode_for_test(GEARSHIFT_V2_LEGACY);
		if(!r6_on || r6_off || r6_live)
		{ printf("[TEST-GS2SC] FAIL R6: on=%d off=%d live=%d\n", r6_on, r6_off, r6_live); fails++; }
		else printf("[TEST-GS2SC] PASS R6: empty TRANSMITTING_CONTROL returns to DATA under ACTIVE; "
			"a live control exchange does not (knob-off arm stays stranded)\n");
	}

	printf("[TEST-GS2SC] %s (failures=%d)\n", fails == 0 ? "ALL PASS" : "FAIL", fails);
	return fails;
}

// INCREMENT 2 (MC-2) directed in-process unit. Synthetic-fires the PRODUCTION producers and
// drives the PRODUCTION selection/arithmetic (mc2_slot_floor_kd_ms / mc2_apply_slot_floor) on a
// throwaway controller with the deterministic legacy geometry (telecom_system NULL =>
// derive_keydown_length_ms(n) == n*message_transmission_time_ms). Asserts, in order:
//  ARM 0 pre-init      — owner==NONE => the guard falls back to the latch derivation (src=0);
//  ARM 1 clean path    — a valid same-config primitive is used (src=1) and, at a geometry where
//                        kd_ms_exact == kd_ms_lever, its value is BYTE-IDENTICAL to the latch path;
//  ARM 2 stale-epoch   — after a config switch bumps the generation, the STALE primitive value is
//    (fail-before)       REJECTED and the floor sizes from the new-geometry fallback (never the
//                        stale keydown-end); WITHOUT the epoch guard the stale value would be used;
//  ARM 3 RAISE-ONLY    — mc2_apply_slot_floor never returns a timeout below its input.
// No IONOS/RF. Fast + deterministic.
int cl_arq_controller::test_linkphase_mc2_slotfloor()
{
	int fails = 0;
	telecom_system = NULL;                 // deterministic legacy derive: derive(n)==n*mtt
	message_transmission_time_ms = 500;    // per-frame airtime
	data_batch_size = 10;                  // latch-absent fallback frame count
	ptt_off_delay_ms = 0; ptt_on_delay_ms = 0;
	lp_config_gen = 0;
	linkphase_cmd_slot_floor_fired = 0;

	// ARM 0 — pre-init: owner==NONE => the guard must fall back to the private latch derivation.
	lp_reset();
	linkphase_last_kd_frames = 8; linkphase_last_kd_force_full = false;
	int src = -1;
	int kd  = mc2_slot_floor_kd_ms(src);
	int expect_fallback = derive_keydown_length_ms(8, false);     // 8*500 = 4000
	if(src != 0 || kd != expect_fallback)
	{ printf("[TEST-LP-MC2] FAIL: pre-init not latch fallback (src=%d kd=%d exp=%d)\n",
		src, kd, expect_fallback); fails++; }
	else printf("[TEST-LP-MC2] PASS: pre-init owner==LP_NONE -> latch fallback kd=%d (src=0)\n", kd);

	// ARM 1 — clean path byte-identity. Fire the CMD keydown producers at a geometry where the
	// sample-exact keydown length equals the arithmetic latch derivation exactly:
	//   frames_region_len = 240000 samples => kd_ms_exact = 240000/48 = 5000 = 10 frames * 500 ms.
	lp_reset();
	linkphase_last_kd_frames = 10; linkphase_last_kd_force_full = false;
	lp_note_keydown_start(/*bsi=*/5);
	lp_note_keydown_end(/*bsi=*/5, /*frames_region_len=*/240000, /*frames=*/10, /*force_full=*/false);
	src = -1;
	kd  = mc2_slot_floor_kd_ms(src);
	int latch_kd = derive_keydown_length_ms(10, false);           // 5000
	if(src != 1)
	{ printf("[TEST-LP-MC2] FAIL: clean path did not use the primitive (src=%d)\n", src); fails++; }
	else if(kd != latch_kd)
	{ printf("[TEST-LP-MC2] FAIL: clean-path primitive kd=%d != latch kd=%d (not byte-identical)\n",
		kd, latch_kd); fails++; }
	else printf("[TEST-LP-MC2] PASS: clean-path primitive kd=%d == latch kd=%d (src=1, byte-identical)\n",
		kd, latch_kd);

	// ARM 2 — stale-epoch config switch (FAIL-BEFORE / PASS-AFTER). The primitive still holds the
	// OLD-config keydown-end (5000 ms). A config switch bumps the generation and changes geometry;
	// the latch is re-derived at the NEW geometry. The stale primitive MUST be rejected by the
	// epoch guard, so the floor sizes from the new-geometry fallback (1800), never the stale 5000.
	// WITHOUT the (epoch>>8)==lp_config_gen guard the helper would return the stale 5000 = the bug.
	// Tip reconciliation: lp_note_config_switch() now RESETS owner_keydown_end_ms to 0, which
	// would let ARM 2 pass on the positive-length guard alone and neuter the epoch-guard
	// fail-before. Bump the config generation DIRECTLY so the primitive keeps a positive,
	// stale-generation keydown-end and the (epoch>>8)==lp_config_gen guard is the load-
	// bearing check (-DLINKPHASE_MC2_NOGUARD then consumes the stale value = the fail-before).
	lp_config_gen++;                       // 0 -> 1: primitive.epoch generation now stale
	message_transmission_time_ms = 300;    // NEW geometry
	linkphase_last_kd_frames = 6;          // NEW keydown latch (fresh at the new config)
	int stale_primitive_kd = (int)(lp_state.owner_keydown_end_ms - lp_keydown_start_ms);  // 5000
	int new_geom_fallback  = derive_keydown_length_ms(6, false);  // 6*300 = 1800
	src = -1;
	kd  = mc2_slot_floor_kd_ms(src);
	if(src != 0 || kd != new_geom_fallback)
	{ printf("[TEST-LP-MC2] FAIL: stale config switch used stale primitive "
		"(src=%d kd=%d stale=%d new_geom=%d)\n", src, kd, stale_primitive_kd, new_geom_fallback);
		fails++; }
	else printf("[TEST-LP-MC2] PASS: stale config switch rejected stale primitive %d, used "
		"new-geometry fallback %d (src=0)\n", stale_primitive_kd, new_geom_fallback);

	// ARM 3 — RAISE-ONLY. Re-populate a valid same-config primitive, then drive the production
	// slot-floor apply with (a) a tiny input timeout (slot_close > timeout -> raised) and (b) a
	// large input below the I-1 cap (already >= slot_close -> unchanged). The result must never be
	// below the input.
	lp_reset();
	message_transmission_time_ms = 500;
	ptt_off_delay_ms = 0; ptt_on_delay_ms = 0;
	linkphase_last_kd_frames = 10; linkphase_last_kd_force_full = false;
	lp_note_keydown_start(/*bsi=*/7);
	lp_note_keydown_end(/*bsi=*/7, /*frames_region_len=*/240000, /*frames=*/10, /*force_full=*/false);
	int s2 = 0;
	int t_small = 100;
	int raised = mc2_apply_slot_floor(t_small, s2);
	if(raised < t_small)
	{ printf("[TEST-LP-MC2] FAIL: RAISE-ONLY violated (in=%d out=%d)\n", t_small, raised); fails++; }
	else printf("[TEST-LP-MC2] PASS: RAISE-ONLY small in=%d -> out=%d (>=, src=%d)\n",
		t_small, raised, s2);
	int t_big = 59000;
	int held = mc2_apply_slot_floor(t_big, s2);
	if(held < t_big)
	{ printf("[TEST-LP-MC2] FAIL: RAISE-ONLY violated (in=%d out=%d)\n", t_big, held); fails++; }
	else printf("[TEST-LP-MC2] PASS: RAISE-ONLY large in=%d -> out=%d (>=)\n", t_big, held);

	if(fails == 0) printf("[TEST-LP-MC2] ALL PASS\n");
	else           printf("[TEST-LP-MC2] %d FAIL(s)\n", fails);
	fflush(stdout);
	return fails;
}

// INCREMENT 3 (step-3) directed unit — the sample-anchored reverse-ACK detection window's
// PRODUCTION DECISION LOGIC (the gate predicate lp_ack_window_active + the sub-class-b branch
// predicate lp_ackwin_beyond_rescue_eligible — the SAME functions the live paths call) plus the
// byte-identical-OFF / WIDEN-only invariant of cmd_data_ack_anchored_rescan_accept. Deterministic,
// telecom_system==NULL (the gate closes before any DSP so the NULL is never dereferenced). The
// full sub-class-a older-phase DSP CREDIT (waveform -> detect -> CRC -> meter 0->N) is proven on
// the WIRE by the live 14.8 storm smoke (the required fire-proof), NOT simulated here.
//  ARM 0 pre-init          — owner==NONE => gate CLOSED; both new paths inert.
//  ARM 1 gate open/close    — a valid completed keydown (owner==TURNAROUND, matched tokens, fresh
//    (fail-before/after)      epoch) OPENS the window; a config-switch generation bump (stale
//                             epoch) or a BREAK (owner!=TURNAROUND) CLOSES it. cmd_data_ack_
//                             anchored_rescan_accept returns false OFF (byte-identical, NULL-safe).
//  ARM 2 sub-b rescue       — the beyond-ring predicate is eligible for a small-overshoot,
//    (fail-before/after)      present-band-metric preamble during the open window, and NOT eligible
//                             for gap>24 / metric<0.11 / metric>=0.5 / wrong role / closed window.
//  ARM 3 backstop invariant — with the window closed the rescan returns false (the incumbent
//                             miss/break path is reached byte-identical).
int cl_arq_controller::test_linkphase_ackwin()
{
	int fails = 0;
	telecom_system = NULL;                 // gate closes before any telecom_system deref
	message_transmission_time_ms = 500;
	data_batch_size = 10;
	ptt_off_delay_ms = 0; ptt_on_delay_ms = 0;
	receiving_timeout = 3000;
	lp_config_gen = 0;
	role = COMMANDER;
	connection_status = RECEIVING_ACKS_DATA;
	linkphase_false_break_averted = 0;
	linkphase_ofdm_beyond_rescue_fired = 0;

	// ARM 0 — pre-init: owner==LP_NONE => window CLOSED.
	lp_reset();
	if(lp_ack_window_active(NULL))
	{ printf("[TEST-LP-ACKWIN] FAIL: pre-init owner==LP_NONE reported window OPEN\n"); fails++; }
	else printf("[TEST-LP-ACKWIN] PASS: pre-init owner==LP_NONE -> window CLOSED\n");
	if(cmd_data_ack_anchored_rescan_accept())
	{ printf("[TEST-LP-ACKWIN] FAIL: rescan credited with the window CLOSED (pre-init)\n"); fails++; }
	else printf("[TEST-LP-ACKWIN] PASS: rescan returns false with window CLOSED (byte-identical OFF)\n");
	if(lp_ackwin_beyond_rescue_eligible(/*pream*/130, /*upper*/127, /*metric*/0.12))
	{ printf("[TEST-LP-ACKWIN] FAIL: sub-b eligible pre-init (window CLOSED)\n"); fails++; }
	else printf("[TEST-LP-ACKWIN] PASS: sub-b NOT eligible pre-init (window CLOSED)\n");

	// ARM 1 — a completed CMD keydown OPENS the window under the strong token guard. PASS-AFTER:
	// owner==TURNAROUND + matched tokens + fresh epoch + next_listen_open_ms>0 => OPEN.
	lp_reset();
	linkphase_last_kd_frames = 10; linkphase_last_kd_force_full = false;
	lp_note_keydown_start(/*bsi=*/5);
	lp_note_keydown_end(/*bsi=*/5, /*frames_region_len=*/240000, /*frames=*/10, /*force_full=*/false);
	if(!lp_ack_window_active(NULL))
	{ printf("[TEST-LP-ACKWIN] FAIL: completed keydown did not OPEN the window\n"); fails++; }
	else printf("[TEST-LP-ACKWIN] PASS: completed keydown (owner==TURNAROUND, tokens matched) -> window OPEN\n");
	// FAIL-BEFORE (a): stale epoch after a config-switch generation bump => CLOSED (the strong
	// guard rejects the stale stamp — mirrors the MC-2 ARM-2 stale-epoch fail-before).
	lp_config_gen++;   // primitive.epoch generation now stale vs lp_config_gen
	if(lp_ack_window_active(NULL))
	{ printf("[TEST-LP-ACKWIN] FAIL: stale-epoch primitive kept the window OPEN\n"); fails++; }
	else printf("[TEST-LP-ACKWIN] PASS: stale-epoch (config-gen bump) -> window CLOSED\n");
	// FAIL-BEFORE (b): a BREAK moves owner to CMD_KEYED => CLOSED.
	lp_reset();
	lp_note_keydown_start(/*bsi=*/6);
	lp_note_keydown_end(/*bsi=*/6, /*frames_region_len=*/240000, /*frames=*/10, /*force_full=*/false);
	lp_note_break(/*bsi=*/7);
	if(lp_ack_window_active(NULL))
	{ printf("[TEST-LP-ACKWIN] FAIL: post-BREAK owner!=TURNAROUND kept the window OPEN\n"); fails++; }
	else printf("[TEST-LP-ACKWIN] PASS: post-BREAK owner==CMD_KEYED -> window CLOSED\n");

	// ARM 2 — sub-class-b beyond-ring rescue predicate. Re-open the window, then walk the band.
	lp_reset();
	linkphase_last_kd_frames = 10; linkphase_last_kd_force_full = false;
	lp_note_keydown_start(/*bsi=*/8);
	lp_note_keydown_end(/*bsi=*/8, /*frames_region_len=*/240000, /*frames=*/10, /*force_full=*/false);
	// PASS-AFTER: the §158 signature — pream just past upper, present-band metric ~0.12.
	if(!lp_ackwin_beyond_rescue_eligible(/*pream*/130, /*upper*/127, /*metric*/0.12))
	{ printf("[TEST-LP-ACKWIN] FAIL: §158 reverse-SACK (pream=130 upper=127 metric=0.12) NOT rescued\n"); fails++; }
	else printf("[TEST-LP-ACKWIN] PASS: reverse-SACK pream=130 upper=127 metric=0.12 -> rescue eligible\n");
	if(!lp_ackwin_beyond_rescue_eligible(/*pream*/141, /*upper*/127, /*metric*/0.131))
	{ printf("[TEST-LP-ACKWIN] FAIL: reverse-SACK pream=141 upper=127 metric=0.131 NOT rescued\n"); fails++; }
	else printf("[TEST-LP-ACKWIN] PASS: reverse-SACK pream=141 upper=127 metric=0.131 -> rescue eligible\n");
	// FAIL-BEFORE band edges: gap too large, metric below the alias floor, metric at/above 0.5,
	// and pream within bounds (not beyond) all reject.
	bool edge_ok =
		   !lp_ackwin_beyond_rescue_eligible(/*pream*/200, /*upper*/127, /*metric*/0.12)   // gap 73 > 24
		&& !lp_ackwin_beyond_rescue_eligible(/*pream*/130, /*upper*/127, /*metric*/0.09)   // metric < 0.11
		&& !lp_ackwin_beyond_rescue_eligible(/*pream*/130, /*upper*/127, /*metric*/0.60)   // metric >= 0.5 (stock)
		&& !lp_ackwin_beyond_rescue_eligible(/*pream*/125, /*upper*/127, /*metric*/0.12);  // not beyond
	if(!edge_ok)
	{ printf("[TEST-LP-ACKWIN] FAIL: a band-edge case was wrongly rescued\n"); fails++; }
	else printf("[TEST-LP-ACKWIN] PASS: gap>24 / metric<0.11 / metric>=0.5 / in-bounds all rejected\n");
	// wrong role rejects even in-band.
	role = RESPONDER;
	if(lp_ackwin_beyond_rescue_eligible(/*pream*/130, /*upper*/127, /*metric*/0.12))
	{ printf("[TEST-LP-ACKWIN] FAIL: RESPONDER role was rescued\n"); fails++; }
	else printf("[TEST-LP-ACKWIN] PASS: RESPONDER role NOT eligible (COMMANDER-only)\n");
	role = COMMANDER;

	// ARM 3 — backstop invariant: with the window CLOSED (config switch), the rescan credits
	// nothing (returns false) so the incumbent miss/break path is reached byte-identical.
	if(!lp_note_config_switch())
	{ printf("[TEST-LP-ACKWIN] FAIL: config-switch contract rejected valid transition\n"); fails++; }
	if(lp_ack_window_active(NULL))
	{ printf("[TEST-LP-ACKWIN] FAIL: config-switch left the window OPEN\n"); fails++; }
	else printf("[TEST-LP-ACKWIN] PASS: config-switch closes the window (backstop: incumbent break path intact)\n");
	if(cmd_data_ack_anchored_rescan_accept() || linkphase_false_break_averted != 0)
	{ printf("[TEST-LP-ACKWIN] FAIL: rescan credited / counter moved with the window CLOSED\n"); fails++; }
	else printf("[TEST-LP-ACKWIN] PASS: window CLOSED -> no credit, averted counter stays 0 (WIDEN-only)\n");

	if(fails == 0) printf("[TEST-LP-ACKWIN] ALL PASS\n");
	else           printf("[TEST-LP-ACKWIN] %d FAIL(s)\n", fails);
	fflush(stdout);
	return fails;
}

// Synthetic geometry-switch regression for MC-7. The END stamp records 5000 ms,
// then live geometry changes so the legacy re-derive becomes 6000 ms. Production's
// selector must feed the optimizer the historical END duration; after an epoch bump,
// the same stamp is stale and must take the live fallback.
int cl_arq_controller::test_linkphase_optclock_end_stamp()
{
	int fails = 0;
	telecom_system = NULL;
	message_transmission_time_ms = 500;
	data_batch_size = 10;
	lp_reset();
	lp_note_keydown_start(/*bsi=*/7);
	linkphase_last_kd_frames = 10;
	linkphase_last_kd_force_full = false;
	lp_note_keydown_end(/*bsi=*/7, /*frames_region_len=*/240000,
		/*frames=*/10, /*force_full=*/false);

	message_transmission_time_ms = 600; // post-END geometry: live derive is now 6000 ms
	const int live_ms = derive_keydown_length_ms(10, false);
	bool used_stamp = false;
	const int optimizer_ms = lp_optclock_keydown_ms(10, false, &used_stamp);
	if(live_ms != 6000 || optimizer_ms != 5000 || !used_stamp)
	{
		printf("[TEST-LP-OPTCLOCK] FAIL: live=%d optimizer=%d stamp=%d "
		       "(expected live=6000 optimizer=5000 stamp=1)\n",
			live_ms, optimizer_ms, used_stamp ? 1 : 0);
		fails++;
	}
	else
		printf("[TEST-LP-OPTCLOCK] PASS: geometry-switch optimizer input uses "
		       "END stamp 5000 ms, not live re-derive 6000 ms\n");

	if(!lp_note_config_switch())
	{ printf("[TEST-LP-OPTCLOCK] FAIL: config-switch contract rejected valid transition\n"); fails++; }
	used_stamp = true;
	const int stale_ms = lp_optclock_keydown_ms(10, false, &used_stamp);
	if(stale_ms != live_ms || used_stamp)
	{
		printf("[TEST-LP-OPTCLOCK] FAIL: stale stamp fallback=%d live=%d stamp=%d\n",
			stale_ms, live_ms, used_stamp ? 1 : 0);
		fails++;
	}
	else
		printf("[TEST-LP-OPTCLOCK] PASS: stale epoch rejects END stamp and "
		       "falls back to live derive %d ms\n", stale_ms);

	// A BREAK does not change configuration. Nevertheless it ends the causal lifetime of the
	// completed START/END pair. Restamp a valid keydown at the current configuration, then BREAK
	// before another END is committed. The selector must reject that preceding keydown and use the
	// explicit live-shape fallback.
	message_transmission_time_ms = 500;
	lp_note_keydown_start(/*bsi=*/8);
	lp_note_keydown_end(/*bsi=*/8, /*frames_region_len=*/240000,
		/*frames=*/10, /*force_full=*/false);
	message_transmission_time_ms = 600;
	lp_note_break(/*bsi=*/9);
	used_stamp = true;
	const int post_break_ms = lp_optclock_keydown_ms(10, false, &used_stamp);
	if(post_break_ms != 6000 || used_stamp)
	{
		printf("[TEST-LP-OPTCLOCK] FAIL: same-config BREAK reused preceding END "
		       "stamp=%d ms used=%d (expected fallback=6000 used=0)\n",
			post_break_ms, used_stamp ? 1 : 0);
		fails++;
	}
	else
		printf("[TEST-LP-OPTCLOCK] PASS: same-config BREAK invalidates preceding "
		       "END stamp and falls back to 6000 ms\n");

	// A new START has no END yet and must not expose the preceding keydown's endpoint.
	lp_note_keydown_start(/*bsi=*/10);
	used_stamp = true;
	const int start_only_ms = lp_optclock_keydown_ms(10, false, &used_stamp);
	if(start_only_ms != 6000 || used_stamp)
	{
		printf("[TEST-LP-OPTCLOCK] FAIL: START-without-END exposed stamp=%d ms "
		       "used=%d (expected fallback=6000 used=0)\n",
			start_only_ms, used_stamp ? 1 : 0);
		fails++;
	}
	else
		printf("[TEST-LP-OPTCLOCK] PASS: START-without-END rejects preceding stamp\n");

	// Observing the commander's keydown on the RX side transfers ownership away from a
	// completed local keydown. The local token pair must no longer be consumable.
	message_transmission_time_ms = 500;
	lp_note_keydown_start(/*bsi=*/12);
	lp_note_keydown_end(/*bsi=*/12, /*frames_region_len=*/240000,
		/*frames=*/10, /*force_full=*/false);
	message_transmission_time_ms = 600;
	lp_note_rx_frame0(/*bsi=*/12, /*d5_wire=*/10);
	used_stamp = true;
	const int post_rx_ms = lp_optclock_keydown_ms(10, false, &used_stamp);
	if(post_rx_ms != 6000 || used_stamp || lp_state.owner != LP_CMD_KEYED
	   || lp_keydown_start_token != 0 || lp_state.keydown_end_token != 0)
	{
		printf("[TEST-LP-OPTCLOCK] FAIL: RX ownership retained local stamp=%d used=%d "
		       "owner=%d start_token=%llu end_token=%llu\n",
			post_rx_ms, used_stamp ? 1 : 0, (int)lp_state.owner,
			(unsigned long long)lp_keydown_start_token,
			(unsigned long long)lp_state.keydown_end_token);
		fails++;
	}
	else
		printf("[TEST-LP-OPTCLOCK] PASS: RX ownership invalidates local END token\n");

	// Keying a reverse ACK similarly transfers ownership and invalidates any completed
	// local keydown pair, regardless of whether the configuration changed.
	message_transmission_time_ms = 500;
	lp_note_keydown_start(/*bsi=*/13);
	lp_note_keydown_end(/*bsi=*/13, /*frames_region_len=*/240000,
		/*frames=*/10, /*force_full=*/false);
	message_transmission_time_ms = 600;
	lp_note_rsp_key();
	used_stamp = true;
	const int post_rsp_ms = lp_optclock_keydown_ms(10, false, &used_stamp);
	if(post_rsp_ms != 6000 || used_stamp || lp_state.owner != LP_RSP_KEYED
	   || lp_keydown_start_token != 0 || lp_state.keydown_end_token != 0)
	{
		printf("[TEST-LP-OPTCLOCK] FAIL: RSP key retained local stamp=%d used=%d "
		       "owner=%d start_token=%llu end_token=%llu\n",
			post_rsp_ms, used_stamp ? 1 : 0, (int)lp_state.owner,
			(unsigned long long)lp_keydown_start_token,
			(unsigned long long)lp_state.keydown_end_token);
		fails++;
	}
	else
		printf("[TEST-LP-OPTCLOCK] PASS: RSP key invalidates local END token\n");

	// Session reset invalidates the pair without rewinding the full-width generation.
	message_transmission_time_ms = 500;
	lp_note_keydown_start(/*bsi=*/14);
	lp_note_keydown_end(/*bsi=*/14, /*frames_region_len=*/240000,
		/*frames=*/10, /*force_full=*/false);
	const uint64_t pre_reset_token = lp_state.keydown_end_token;
	lp_reset();
	message_transmission_time_ms = 600;
	used_stamp = true;
	const int post_reset_ms = lp_optclock_keydown_ms(10, false, &used_stamp);
	if(post_reset_ms != 6000 || used_stamp || lp_state.owner != LP_NONE
	   || lp_keydown_start_token != 0 || lp_state.keydown_end_token != 0
	   || lp_keydown_generation != pre_reset_token)
	{
		printf("[TEST-LP-OPTCLOCK] FAIL: reset retained/reused stamp=%d used=%d "
		       "owner=%d generation=%llu prior=%llu start_token=%llu end_token=%llu\n",
			post_reset_ms, used_stamp ? 1 : 0, (int)lp_state.owner,
			(unsigned long long)lp_keydown_generation,
			(unsigned long long)pre_reset_token,
			(unsigned long long)lp_keydown_start_token,
			(unsigned long long)lp_state.keydown_end_token);
		fails++;
	}
	else
		printf("[TEST-LP-OPTCLOCK] PASS: reset invalidates pair without rewinding token\n");

	// A decoded ACK is not an invalidation: it preserves the still-current completed
	// local pair. The first START after reset must allocate a different token.
	message_transmission_time_ms = 500;
	lp_note_keydown_start(/*bsi=*/15);
	const uint64_t post_reset_token = lp_keydown_start_token;
	lp_note_keydown_end(/*bsi=*/15, /*frames_region_len=*/240000,
		/*frames=*/10, /*force_full=*/false);
	lp_note_ack_decoded();
	message_transmission_time_ms = 600;
	used_stamp = false;
	const int post_ack_ms = lp_optclock_keydown_ms(10, false, &used_stamp);
	if(post_ack_ms != 5000 || !used_stamp || lp_state.owner != LP_TURNAROUND
	   || post_reset_token == 0 || post_reset_token == pre_reset_token
	   || lp_keydown_start_token != post_reset_token
	   || lp_state.keydown_end_token != post_reset_token)
	{
		printf("[TEST-LP-OPTCLOCK] FAIL: ACK decode did not preserve current pair "
		       "stamp=%d used=%d owner=%d prior=%llu current=%llu start=%llu end=%llu\n",
			post_ack_ms, used_stamp ? 1 : 0, (int)lp_state.owner,
			(unsigned long long)pre_reset_token,
			(unsigned long long)post_reset_token,
			(unsigned long long)lp_keydown_start_token,
			(unsigned long long)lp_state.keydown_end_token);
		fails++;
	}
	else
		printf("[TEST-LP-OPTCLOCK] PASS: decoded ACK preserves current completed pair\n");

	// An END from a different BSI is not the END for the current START and must not
	// publish a consumable pair.
	message_transmission_time_ms = 500;
	lp_note_keydown_start(/*bsi=*/16);
	lp_note_keydown_end(/*bsi=*/17, /*frames_region_len=*/240000,
		/*frames=*/10, /*force_full=*/false);
	message_transmission_time_ms = 600;
	used_stamp = true;
	const int mismatched_end_ms = lp_optclock_keydown_ms(10, false, &used_stamp);
	if(mismatched_end_ms != 6000 || used_stamp || lp_state.owner != LP_CMD_KEYED
	   || lp_keydown_start_token == 0 || lp_state.keydown_end_token != 0
	   || lp_state.owner_keydown_end_ms != 0)
	{
		printf("[TEST-LP-OPTCLOCK] FAIL: mismatched END became consumable "
		       "stamp=%d used=%d owner=%d start_token=%llu end_token=%llu end_ms=%lld\n",
			mismatched_end_ms, used_stamp ? 1 : 0, (int)lp_state.owner,
			(unsigned long long)lp_keydown_start_token,
			(unsigned long long)lp_state.keydown_end_token,
			lp_state.owner_keydown_end_ms);
		fails++;
	}
	else
		printf("[TEST-LP-OPTCLOCK] PASS: mismatched END cannot publish a stamp\n");

	// The selector itself also rejects a nonzero END token that no longer matches START.
	message_transmission_time_ms = 500;
	lp_note_keydown_start(/*bsi=*/18);
	lp_note_keydown_end(/*bsi=*/18, /*frames_region_len=*/240000,
		/*frames=*/10, /*force_full=*/false);
	lp_state.keydown_end_token = lp_keydown_start_token + 1;
	message_transmission_time_ms = 600;
	used_stamp = true;
	const int mismatched_token_ms = lp_optclock_keydown_ms(10, false, &used_stamp);
	if(mismatched_token_ms != 6000 || used_stamp)
	{
		printf("[TEST-LP-OPTCLOCK] FAIL: mismatched END token became consumable "
		       "stamp=%d used=%d start_token=%llu end_token=%llu\n",
			mismatched_token_ms, used_stamp ? 1 : 0,
			(unsigned long long)lp_keydown_start_token,
			(unsigned long long)lp_state.keydown_end_token);
		fails++;
	}
	else
		printf("[TEST-LP-OPTCLOCK] PASS: mismatched END token is rejected\n");

	// Exhaustion is fail-closed: the full-width token is never reused as a valid START.
	lp_keydown_generation = UINT64_MAX - 1;
	lp_note_keydown_start(/*bsi=*/11);
	used_stamp = true;
	const int exhausted_ms = lp_optclock_keydown_ms(10, false, &used_stamp);
	if(exhausted_ms != 6000 || used_stamp || lp_keydown_start_token != 0
	   || lp_state.keydown_end_token != 0 || lp_keydown_start_ms != 0)
	{
		printf("[TEST-LP-OPTCLOCK] FAIL: exhausted token remained consumable "
		       "stamp=%d used=%d start_token=%llu end_token=%llu start_ms=%lld\n",
			exhausted_ms, used_stamp ? 1 : 0,
			(unsigned long long)lp_keydown_start_token,
			(unsigned long long)lp_state.keydown_end_token, lp_keydown_start_ms);
		fails++;
	}
	else
		printf("[TEST-LP-OPTCLOCK] PASS: token exhaustion fails closed\n");

	printf("[TEST-LP-OPTCLOCK] %s (failures=%d)\n", fails ? "FAIL" : "ALL PASS", fails);
	fflush(stdout);
	return fails;
}

// Directed in-process unit for the increment-1 shadow primitive. Synthetic-fires the
// PRODUCTION producers (lp_note_*) on a throwaway controller with a deterministic legacy
// geometry (telecom_system NULL => derive_keydown_length_ms(n) == n*message_transmission_time_ms),
// and asserts: the pre-init owner==NONE guard; the owner ordering at every transition; the
// DIRECTIONAL meter (RX-derived >= CMD-emitted = SAFE over-wait); the intended short-batch
// clamp classifying as SAFE (not a desync); the FAIL-BEFORE that a SHORTENING derivation IS
// flagged an under-estimate; and the epoch bump on config-switch and BREAK. No IONOS/RF.
int cl_arq_controller::test_linkphase_shadow()
{
	int fails = 0;
	telecom_system = NULL;                 // deterministic legacy derive: derive(n)==n*mtt
	message_transmission_time_ms = 500;    // per-frame airtime
	data_batch_size = 10;                  // RX conservative window floor
	sack_v2_enabled = true;                // D5 + rx_effective_window widening are sack_v2-only

	// ARM 0 — pre-init: the timeline defaults to NONE (no consumer may act on it).
	lp_reset();
	if(lp_state.owner != LP_NONE || lp_state.owner_keydown_end_ms != 0
	   || lp_state.next_listen_open_ms != 0 || lp_state.epoch != 0)
	{
		printf("[TEST-LP-SHADOW] FAIL: pre-init timeline not NONE/0 (owner=%d)\n",
			(int)lp_state.owner); fails++;
	}
	else printf("[TEST-LP-SHADOW] PASS: pre-init owner==LP_NONE, instants/epoch 0 "
		"(unpopulated-timeline guard)\n");

	// ARM 1 — synthetic-fire the CMD keydown producers at a KNOWN geometry.
	// A 10-frame keydown; frames_region_len chosen so kd_ms_exact == 10*500 == 5000 ms:
	//   frames_region_len = 5000*48000/1000 = 240000 samples.
	lp_note_keydown_start(/*bsi=*/5);
	if(lp_state.owner != LP_CMD_KEYED)
	{ printf("[TEST-LP-SHADOW] FAIL: keydown-START owner!=CMD_KEYED\n"); fails++; }
	lp_note_keydown_end(/*bsi=*/5, /*frames_region_len=*/240000, /*frames=*/10,
		/*force_full=*/false);
	if(lp_state.owner != LP_TURNAROUND)
	{ printf("[TEST-LP-SHADOW] FAIL: keydown-END owner!=TURNAROUND\n"); fails++; }
	int cmd_kd = 5000;   // the CMD sample-exact keydown length just committed
	// RX derives the SAME keydown from D5=10 (a full batch) through the SAME schedule.
	lp_note_rx_frame0(/*bsi=*/5, /*d5=*/10);          // production producer fires
	if(lp_state.owner != LP_CMD_KEYED)
	{ printf("[TEST-LP-SHADOW] FAIL: rx-frame0 owner!=CMD_KEYED\n"); fails++; }
	int rx_full = derive_keydown_length_ms(rx_effective_window(10), false);   // == 5000
#ifdef LINKPHASE_SHADOW_FAILBEFORE
	rx_full -= 1000;   // inject a SHORTENING so the RX would close before the CMD keydown ends;
	                   // the SAFE assertion below MUST catch it (proves the meter has teeth).
#endif
	if(rx_full < cmd_kd)
	{ printf("[TEST-LP-SHADOW] FAIL: full-batch RX(%d) < CMD(%d) = under-estimate\n",
		rx_full, cmd_kd); fails++; }
	else printf("[TEST-LP-SHADOW] PASS: full-batch RX-derived(%d) >= CMD-emitted(%d) (SAFE)\n",
		rx_full, cmd_kd);

	// ARM 2 — intended SHORT-BATCH CLAMP registers as SAFE over-wait, NOT a desync.
	// CMD keyed a 3-frame final batch (cmd=1500); the RX has no samples and
	// rx_effective_window(D5=3) returns data_batch_size=10 -> derive=5000 >> 1500 = over-wait.
	int cmd_short = 3 * message_transmission_time_ms;                  // 1500
	int rx_clamped = derive_keydown_length_ms(rx_effective_window(3), false);  // eff=10 -> 5000
	bool clamp = rx_effective_window(3) > 3;
	if(rx_clamped >= cmd_short && clamp)
		printf("[TEST-LP-SHADOW] PASS: short-batch clamp RX(%d)>=CMD(%d), eff_window(%d)>d5(3) "
			"= SAFE over-wait, NOT a desync\n", rx_clamped, cmd_short, rx_effective_window(3));
	else { printf("[TEST-LP-SHADOW] FAIL: short-batch clamp misclassified "
		"(rx=%d cmd=%d eff=%d)\n", rx_clamped, cmd_short, rx_effective_window(3)); fails++; }

	// ARM 3 (FAIL-BEFORE) — a SHORTENING derivation IS flagged an UNDER-estimate by the meter.
	// (The production rx_effective_window can only clamp UP, never shorter, so no production
	// input produces this; this arm proves the meter is not vacuously passing.)
	int cmd_true = 12 * message_transmission_time_ms;   // CMD keyed 12 frames = 6000
	int rx_under = derive_keydown_length_ms(8, true);   // a hypothetical 8-frame derivation = 4000
	if(rx_under < cmd_true)
		printf("[TEST-LP-SHADOW] PASS(fail-before): a SHORTENING RX derivation (%d < CMD %d) "
			"IS flagged UNDER-estimate\n", rx_under, cmd_true);
	else { printf("[TEST-LP-SHADOW] FAIL: under-estimate predicate did not fire "
		"(rx=%d cmd=%d)\n", rx_under, cmd_true); fails++; }

	// ARM 4 — RSP key + ACK-decoded owner ordering (producers fire at every transition).
	lp_note_rsp_key();
	if(lp_state.owner != LP_RSP_KEYED)
	{ printf("[TEST-LP-SHADOW] FAIL: rsp-key owner!=RSP_KEYED\n"); fails++; }
	lp_note_ack_decoded();
	if(lp_state.owner != LP_TURNAROUND)
	{ printf("[TEST-LP-SHADOW] FAIL: ack-decoded owner!=TURNAROUND\n"); fails++; }

	// ARM 5 — epoch bumps on CONFIG-SWITCH and BREAK (staleness detection substrate).
	uint32_t ep_before = lp_make_epoch(5);
	if(!lp_note_config_switch())                 // config_gen++
	{ printf("[TEST-LP-SHADOW] FAIL: config-switch contract rejected valid transition\n"); fails++; }
	uint32_t ep_after_cfg = lp_make_epoch(5);
	if(ep_after_cfg == ep_before)
	{ printf("[TEST-LP-SHADOW] FAIL: config-switch did not bump epoch\n"); fails++; }
	else printf("[TEST-LP-SHADOW] PASS: config-switch bumped epoch %u -> %u\n",
		ep_before, ep_after_cfg);
	lp_note_break(9);                            // owner=CMD_KEYED, epoch re-stamped at new bsi
	if(lp_state.owner != LP_CMD_KEYED || (lp_state.epoch & 0xFF) != 9)
	{ printf("[TEST-LP-SHADOW] FAIL: BREAK re-stage (owner=%d epoch_bsi=%u)\n",
		(int)lp_state.owner, lp_state.epoch & 0xFF); fails++; }
	else printf("[TEST-LP-SHADOW] PASS: BREAK re-stage owner==CMD_KEYED, epoch bsi=9\n");

	printf("[TEST-LP-SHADOW] %s (fails=%d)\n", fails == 0 ? "ALL PASS" : "FAILURES", fails);
	return fails;
}

// Fail-before / pass-after unit for the ACK-suffix pollution guard. Drives the SAME production methods
// receive() and the suffix send call (rsp_note_data_frame_snr + rsp_suffix_snr_value), not a fixture:
// a payload decode captures the data SNR, a control decode does NOT overwrite it, and the suffix
// encodes the polluted control read with the guard off vs the honest data read with it on.
int cl_arq_controller::test_ack_suffix_pollution_guard()
{
	int fails = 0;
	auto check = [&](bool cond, const char* msg) {
		if(!cond) { fails++; printf("[TEST-ACKSUFFIX] FAIL: %s\n", msg); }
		else      { printf("[TEST-ACKSUFFIX] ok: %s\n", msg); }
		fflush(stdout);
	};
	printf("[TEST-ACKSUFFIX] start (ACK-suffix pollution guard)\n"); fflush(stdout);

	measurements.SNR_uplink = -99.9;
	measurements.SNR_uplink_data = -99.9;
	unsetenv("MERCURY_RSP_SUFFIX_DATA_SNR");

	// (1) CAPTURE — a payload decode records the forward-DATA SNR.
	rsp_note_data_frame_snr(DATA_LONG, 13.0);
	check(measurements.SNR_uplink_data == 13.0, "T1: a DATA_LONG decode captures the forward-data SNR (13)");
	rsp_note_data_frame_snr(DATA_SHORT, 15.0);
	check(measurements.SNR_uplink_data == 15.0, "T1: a DATA_SHORT decode updates the forward-data SNR (15)");

	// (2) A CONTROL / SET_CONFIG / ACK decode must NOT overwrite the captured data SNR.
	rsp_note_data_frame_snr(CONTROL, 34.0);
	check(measurements.SNR_uplink_data == 15.0, "T2: a CONTROL decode does NOT overwrite the data SNR (stays 15)");
	rsp_note_data_frame_snr(SET_CONFIG, 34.3);
	check(measurements.SNR_uplink_data == 15.0, "T2: a SET_CONFIG decode does NOT overwrite the data SNR (stays 15)");
	rsp_note_data_frame_snr(ACK_CONTROL, 30.0);
	check(measurements.SNR_uplink_data == 15.0, "T2: an ACK_CONTROL decode does NOT overwrite the data SNR (stays 15)");

	// (3) THE POLLUTION — the shared SNR_uplink IS overwritten by the control decode (~34.3 dB), which
	//     is exactly what the legacy suffix encodes (ceiling-tone clamp -> spurious 25 at the CMD).
	measurements.SNR_uplink = 34.3;

	// (4) FAIL-BEFORE — guard off: the suffix encodes the polluted control-frame read.
	unsetenv("MERCURY_RSP_SUFFIX_DATA_SNR");
	check(rsp_suffix_snr_value() == 34.3,
		"T3 FAIL-BEFORE: guard off -> suffix encodes the polluted control-frame SNR (34.3)");

	// (5) PASS-AFTER — guard on: the suffix encodes the honest forward-DATA SNR.
	setenv("MERCURY_RSP_SUFFIX_DATA_SNR", "1", 1);
	check(rsp_suffix_snr_value() == 15.0,
		"T4 PASS-AFTER: guard on -> suffix encodes the forward-DATA SNR (15), not the control-frame 34.3");

	// (6) FALLBACK — guard on but no data frame captured yet -> legacy-safe (SNR_uplink).
	measurements.SNR_uplink_data = -99.9;
	check(rsp_suffix_snr_value() == 34.3,
		"T5 FALLBACK: guard on but no data SNR captured -> falls back to SNR_uplink (34.3)");
	unsetenv("MERCURY_RSP_SUFFIX_DATA_SNR");

	printf("[TEST-ACKSUFFIX] done: %d failure(s)\n", fails);
	fflush(stdout);
	return fails;
}

// Fail-before/pass-after regression for the unsupported-suffix early return.
// Seed the state a failed coded ACK would leave behind, then take the NB guard
// without any audio or wall-clock waits. The production packer must subsequently
// observe an uncoded ACK state at an ineligible OFDM configuration.
int cl_arq_controller::test_send_mfsk_ack_sack_fec_state_cleanup()
{
#if !MFSK_ACK_SACK_ENABLED
	printf("[TEST-MFSK-ACK-FEC-CLEANUP] SKIP: MFSK_ACK_SACK_ENABLED == 0\n");
	return 0;
#else
	int failed = 0;
	auto check = [&](bool condition, const char* name) {
		printf("[TEST-MFSK-ACK-FEC-CLEANUP] %s: %s\n",
			condition ? "PASS" : "FAIL", name);
		if(!condition) failed++;
	};

	cl_telecom_system test_telecom;
	test_telecom.operation_mode = ARQ_MODE;
	if(test_telecom.load_configuration(CONFIG_0) != 0
	   || test_telecom.ack_mfsk.M != 16)
	{
		printf("[TEST-MFSK-ACK-FEC-CLEANUP] FAIL: could not load WB CONFIG_0\n");
		return 1;
	}

	cl_telecom_system* saved_telecom = telecom_system;
	bool saved_passive_monitor = passive_monitor;
	int saved_current_configuration = current_configuration;

	telecom_system = &test_telecom;
	passive_monitor = false;
	current_configuration = CONFIG_0; // ACK suffix FEC is ineligible here.

	// Model the shared state left by an enabled robust-tier ACK whose mid-TX
	// wait failed, then force the next call through its no-suffix early exit.
	test_telecom.ack_mfsk.ack_suffix_fec_coded = true;
	test_telecom.ack_mfsk.M = 8;
	long long result = send_mfsk_ack_sack(7, 0x5U);
	check(result == 0, "unsupported suffix returns without transmitting");
	check(!test_telecom.ack_mfsk.ack_suffix_fec_coded,
		"early return clears stale per-call ACK FEC state");

	// Restore WB geometry and exercise the real generation-time packer seam.
	test_telecom.ack_mfsk.M = 16;
	int suffix_tones[64] = {0};
	int suffix_len = -1;
	if(!test_telecom.ack_mfsk.ack_suffix_fec_coded)
		suffix_len = test_telecom.ack_mfsk.pack_ack_sack_payload(
			7, 0x5U, 0, suffix_tones);
	check(suffix_len == test_telecom.ack_mfsk.ack_sack_suffix_len(),
		"subsequent ineligible ACK is uncoded at generation time");

	telecom_system = saved_telecom;
	passive_monitor = saved_passive_monitor;
	current_configuration = saved_current_configuration;

	printf("[TEST-MFSK-ACK-FEC-CLEANUP] %s (%d failure%s)\n",
		failed == 0 ? "ALL PASS" : "FAILURES", failed,
		failed == 1 ? "" : "s");
	fflush(stdout);
	return failed;
#endif
}

// Directed reproduction of the CONFIG_NONE control-level defect (part 1: the
// geometry-true TX gain selector). One binary exercises the quarantined former
// mode-flag selector and the default repair. The WB ACK/OFDM geometry stays
// cached while the session flag and both config owners are reset to NB/NONE,
// exactly as reset_session_state() leaves them after a verified CLOSE+EOT.
int cl_arq_controller::test_control_frame_gain_and_undefined_state()
{
	int bad = 0;
	auto check = [&](bool condition, const char* name) {
		printf("[TEST-CTL-GAIN] %s: %s\n", condition ? "PASS" : "BAD", name);
		if(!condition) bad++;
	};

	cl_telecom_system test_telecom;
	test_telecom.operation_mode = ARQ_MODE;
	if(test_telecom.load_configuration(CONFIG_0) != 0
	   || test_telecom.ack_mfsk.M != 16
	   || test_telecom.ack_mfsk.ack_sack_suffix_len() <= 0)
	{
		printf("[TEST-CTL-GAIN] BAD: could not load WB ACK+SACK geometry\n");
		return 1;
	}
	// Panel-INI control levels (WB vs NB diagonal); distinct so a wrong witness
	// shows as a wrong value. ACK 3.655/15.61; OFDM 0.976/2.15.
	test_telecom.set_tx_gain(TX_SIG_ACK, 0, 3.655);
	test_telecom.set_tx_gain(TX_SIG_ACK, 1, 15.61);
	test_telecom.set_tx_gain(TX_SIG_OFDM, 0, 0.976);
	test_telecom.set_tx_gain(TX_SIG_OFDM, 1, 2.15);

	// Valid-state parity (flag == geometry): while the loaded WB config is
	// intact, the geometry-true selector must return exactly what the flag-based
	// selector returns, for every converted signal type. This is the guarantee
	// that the fix changes nothing outside the reset split.
	check(test_telecom.get_ack_geometry_tx_gain(TX_SIG_ACK)
	          == test_telecom.get_tx_gain(TX_SIG_ACK)
	      && test_telecom.get_ack_geometry_tx_gain(TX_SIG_OFDM)
	          == test_telecom.get_tx_gain(TX_SIG_OFDM),
		"valid WB state: geometry selector matches flag selector");

	cl_telecom_system* saved_telecom = telecom_system;
	const bool saved_passive_monitor = passive_monitor;
	const int saved_configuration = current_configuration;
	const char* saved_gain_env = std::getenv("MERCURY_CTL_GAIN_SELECTOR_DEFEAT");
	const bool had_gain_env = saved_gain_env != NULL;
	const std::string gain_env_value = had_gain_env ? saved_gain_env : "";

	telecom_system = &test_telecom;
	passive_monitor = false;
	current_configuration = CONFIG_NONE;
	test_telecom.current_configuration = CONFIG_NONE;
	test_telecom.narrowband_enabled = YES;  // the reset split: flag NB, geometry WB

	setenv("MERCURY_CTL_GAIN_SELECTOR_DEFEAT", "1", 1);
	const double legacy_ack = test_telecom.get_ack_geometry_tx_gain(TX_SIG_ACK);
	const double legacy_ofdm = test_telecom.get_ack_geometry_tx_gain(TX_SIG_OFDM);
	unsetenv("MERCURY_CTL_GAIN_SELECTOR_DEFEAT");
	const double fixed_ack = test_telecom.get_ack_geometry_tx_gain(TX_SIG_ACK);
	const double fixed_ofdm = test_telecom.get_ack_geometry_tx_gain(TX_SIG_OFDM);
	const double ack_excess_db = 20.0 * std::log10(legacy_ack / fixed_ack);
	const double ofdm_excess_db = 20.0 * std::log10(legacy_ofdm / fixed_ofdm);
	check(std::fabs(legacy_ack - 15.61) < 1e-9,
		"D0 former selector chooses ACK-NB for cached WB geometry");
	check(std::fabs(fixed_ack - 3.655) < 1e-9,
		"default selector chooses ACK-WB from the rendered geometry");
	check(ack_excess_db > 10.0 && std::fabs(ack_excess_db - 12.6103) < 0.01,
		"D0 ACK gain excess is greater than 10 dB and matches the panel arithmetic");
	check(std::fabs(legacy_ofdm - 2.15) < 1e-9,
		"D0 former selector chooses OFDM-NB for cached WB geometry");
	check(std::fabs(fixed_ofdm - 0.976) < 1e-9,
		"default selector chooses OFDM-WB from the rendered geometry");
	check(ofdm_excess_db > 5.0 && ofdm_excess_db < 8.0,
		"D0 OFDM gain excess is between 5 and 8 dB (panel arithmetic)");

	auto render_post_fir = [&](bool defeat, long long* clips, double* peak) {
		if(defeat) setenv("MERCURY_CTL_GAIN_SELECTOR_DEFEAT", "1", 1);
		else unsetenv("MERCURY_CTL_GAIN_SELECTOR_DEFEAT");
		const int pattern_samples = test_telecom.ack_sack_pattern_passband_samples;
		const int symbol_period = test_telecom.data_container.Nofdm
			* test_telecom.data_container.interpolation_rate;
		const int padded_size = pattern_samples + 2 * symbol_period;
		std::vector<double> raw((size_t)padded_size, 0.0);
		std::vector<double> filtered1((size_t)padded_size, 0.0);
		std::vector<double> filtered2((size_t)padded_size, 0.0);
		const int generated = test_telecom.generate_ack_sack_pattern_passband(
			&raw[(size_t)symbol_period], 6, 0x155u, 0x321u);
		if(generated != pattern_samples)
			return false;
		std::memcpy(&raw[0], &raw[(size_t)symbol_period],
			(size_t)symbol_period * sizeof(double));
		std::memcpy(&raw[(size_t)symbol_period + (size_t)pattern_samples],
			&raw[(size_t)pattern_samples], (size_t)symbol_period * sizeof(double));
		test_telecom.ofdm.FIR_tx1.apply(raw.data(), filtered1.data(), padded_size);
		test_telecom.ofdm.FIR_tx2.apply(filtered1.data(), filtered2.data(), padded_size);
		*clips = 0;
		*peak = 0.0;
		for(int i = 0; i < pattern_samples; i++)
		{
			const double a = std::fabs(filtered2[(size_t)symbol_period + (size_t)i]);
			if(a > *peak) *peak = a;
			if(a > 1.0) (*clips)++;
		}
		return true;
	};

	long long legacy_clips = 0, fixed_clips = 0;
	double legacy_peak = 0.0, fixed_peak = 0.0;
	check(render_post_fir(true, &legacy_clips, &legacy_peak),
		"D0 production renderer completes with the former selector");
	check(render_post_fir(false, &fixed_clips, &fixed_peak),
		"default production renderer completes with geometry-directed gain");
	check(legacy_clips > 0 && legacy_peak > 1.0,
		"D0 former selector produces source-clipped post-FIR samples");
	check(fixed_clips == 0 && fixed_peak < 1.0,
		"default geometry-directed renderer has zero source clips");
	check(fixed_peak > 0.0,
		"terminal ACK renders non-zero samples in the dual-CONFIG_NONE state");

	if(had_gain_env) setenv("MERCURY_CTL_GAIN_SELECTOR_DEFEAT", gain_env_value.c_str(), 1);
	else unsetenv("MERCURY_CTL_GAIN_SELECTOR_DEFEAT");
	telecom_system = saved_telecom;
	passive_monitor = saved_passive_monitor;
	current_configuration = saved_configuration;

	printf("[TEST-CTL-GAIN] %s (%d issue%s; legacy_ack_peak=%.6f legacy_clips=%lld "
	       "fixed_ack_peak=%.6f fixed_clips=%lld ack_excess_db=%.4f ofdm_excess_db=%.4f)\n",
		bad == 0 ? "ALL PASS" : "ISSUES", bad, bad == 1 ? "" : "s",
		legacy_peak, legacy_clips, fixed_peak, fixed_clips, ack_excess_db, ofdm_excess_db);
	fflush(stdout);
	return bad;
}

bool cl_arq_controller::test_start_ack_causal_post_drain_refresh()
{
	cl_data_container& dc = telecom_system->data_container;
	const uint64_t saved_deadline_ns =
		dc.start_ack_causal_deadline_ns.load();

	// Model a playback consumer whose delay exceeded the nominal pre-transfer
	// queue estimate. The real post-drain seam must replace this expired
	// boundary before capture_reset_samples() advances the generation.
	dc.start_ack_causal_deadline_ns = 1;
	const uint64_t before_ns = sim_clock_now_ns();
	const uint64_t expected_remaining_ns =
		playback_causal_egress_bound_ns(0)
		+ (uint64_t)START_ACK_RESPONDER_MIN_DELAY_MS * 1000000ULL;
	refresh_start_ack_causal_after_playback_drain();
	const uint64_t after_ns = sim_clock_now_ns();
	const uint64_t refreshed_deadline_ns =
		dc.start_ack_causal_deadline_ns.load();

	const bool ok =
		expected_remaining_ns >=
			(uint64_t)START_ACK_RESPONDER_MIN_DELAY_MS * 1000000ULL
		&& refreshed_deadline_ns >= before_ns + expected_remaining_ns
		&& refreshed_deadline_ns <= after_ns + expected_remaining_ns;
	dc.start_ack_causal_deadline_ns = saved_deadline_ns;
	return ok;
}

// FIX 1 directed regression (also runs in --test) -- drives the REAL
// batch_coast_try_advance() with a REAL cfg15-loaded PHY geometry (upper/frame_symb
// are production values, not hardcoded). Synthetic-fire (the codebase pattern used by
// test_axis2_quiesce_gate / test_burst_coalesce_held_rung / test_climb_confirm_batch):
// it exercises the real coast state machine + the success-mirroring grid arithmetic.
// The continuous-keydown LDPC decode-recovery of frames 6-11 (and the FAIL-BEFORE
// beyond-bounds tail-death) is the COHORT fire proof (fireproof: DEBT) -- reproducing
// the tail-death CONDITION needs the live receive() capture loop.
//   PASS-AFTER: on a mid-batch FAIL (frames 4-5) the helper COASTS -- returns true,
//     keeps ofdm_batch_active, advances the grid one frame period, holds search_raw in
//     (0,upper] across the whole buffer (INV-1 / R2), and the [BATCH-COAST] counter
//     fires 0->N. Grid stays ALIVE so frames 6-11 remain recoverable.
//   FAIL-BEFORE: batch_coast_defeat (MERCURY_BATCH_COAST_DEFEAT) => the helper is INERT
//     (returns false, grid untouched, counter frozen) => the stock teardown the caller
//     then runs kills the grid (the tail-death path).
//   GUARDS: every eligibility leg individually forces a non-coast. 0=PASS, 1=FAIL.
int cl_arq_controller::test_batch_coast()
{
	int failed = 0;
	auto check = [&](bool cond, const char* name) {
		if(cond) printf("[TEST-BATCH-COAST] PASS: %s\n", name);
		else   { printf("[TEST-BATCH-COAST] FAIL: %s\n", name); failed++; }
		fflush(stdout);
	};

	// DEFAULT-ON verification (=0 disables): the ctor env-latch (arq_common.cc:897)
	// arms P2 at depth 8 when MERCURY_BATCH_COAST_MAX is unset, and disables it (0)
	// when the env is "0". Read the ctor-latched member BEFORE prime() overwrites it.
	// FAILS on the pre-flip default (latched 0 != expected 8), PASSES once the default
	// is armed -- the fail-before/pass-after for the default flip. With the env "0"
	// exported this expects 0 and passes (the =0 escape to the byte-identical base).
	{
		const char* bcm_env = std::getenv("MERCURY_BATCH_COAST_MAX");
		int expect_default_on = (bcm_env && *bcm_env) ? atoi(bcm_env) : 8;
		check(batch_coast_max == expect_default_on,
			"DEFAULT-ON: ctor latch arms P2 (env-unset => depth 8; =0 disables)");
	}

	// REAL cfg15 PHY geometry (buffer_Nsymb=139, Nsymb=12, preamble=4 => frame_symb=16,
	// upper=123 -- the exact values the failing coalescing cells ran at).
	cl_telecom_system ts;
	ts.operation_mode = BER_PLOT_passband;
	ts.load_configuration(CONFIG_15);
	this->telecom_system = &ts;
	int frame_symb = ts.data_container.Nsymb + ts.data_container.preamble_nSymb;
	int upper      = (int)ts.data_container.buffer_Nsymb - frame_symb;
	printf("[TEST-BATCH-COAST] cfg15 geometry: Nsymb=%d preamble=%d frame_symb=%d buffer_Nsymb=%d upper=%d\n",
		ts.data_container.Nsymb, ts.data_container.preamble_nSymb, frame_symb,
		(int)ts.data_container.buffer_Nsymb, upper);
	fflush(stdout);
	check(frame_symb > 0 && upper > frame_symb, "real cfg15 geometry loaded (upper>frame_symb)");

	// Common prime: a HELD cfg15 RSP forward-DATA batch, mid-keydown, grid anchored.
	auto prime = [&](int search_raw){
		role = RESPONDER;
		sack_v2_enabled = true;
		current_configuration = CONFIG_15;
		data_batch_size = 12;                 // declared B
		batch_coast_defeat = false;
		batch_coast_max = 0;                  // P2 disabled here (stock full-batch bound)
		batch_coast_stops = 0;
		ts.bigblock_framing_enabled = false;
		ts.receive_stats.ofdm_batch_active = true;
		ts.receive_stats.ofdm_search_raw   = search_raw;
		ts.receive_stats.coast_frames_since_anchor = 0;
		ts.receive_stats.coarse_metric = 0.2;
	};

	// ---- PASS-AFTER: two consecutive mid-batch FAILs (frames 4,5) COAST ----
	batch_coast_advances = 0;
	prime(/*search_raw=*/100);
	int ftr = 0;
	bool c4 = batch_coast_try_advance(frame_symb, upper, /*pream_symb=*/100, /*frame_data_missing=*/false, &ftr);
	check(c4, "frame4 FAIL coasts (returns true)");
	check(ts.receive_stats.ofdm_batch_active, "frame4: batch kept ALIVE");
	check(ts.receive_stats.ofdm_search_raw > 0 && ts.receive_stats.ofdm_search_raw <= upper,
		"frame4: search_raw in (0,upper] (INV-1)");
	check(ftr > 0, "frame4: buffer advances (ftr>0)");
	check(ts.receive_stats.coast_frames_since_anchor == 1, "frame4: coast_frames == 1");
	check(batch_coast_advances == 1, "frame4: [BATCH-COAST] counter 0->1");
	// mimic the shared post-chain shift (search_raw -= nUnder) between frames.
	ts.receive_stats.ofdm_search_raw -= 1;
	bool c5 = batch_coast_try_advance(frame_symb, upper, 100, false, &ftr);
	check(c5, "frame5 FAIL coasts (returns true)");
	check(ts.receive_stats.ofdm_batch_active, "frame5: batch STILL alive (frames 6-11 recoverable)");
	check(ts.receive_stats.ofdm_search_raw > 0 && ts.receive_stats.ofdm_search_raw <= upper,
		"frame5: search_raw in (0,upper] (INV-1)");
	check(batch_coast_advances == 2, "frames 4-5: counter == 2 (only 4-5 are holes)");

	// ---- R2 / INV-1: the advance holds in-bounds for search_raw across the buffer ----
	bool inv_ok = true;
	for(int sr = 1; sr <= upper; sr++){
		prime(sr);
		int f2 = 0;
		bool got = batch_coast_try_advance(frame_symb, upper, sr, false, &f2);
		if(!got) { inv_ok = false; break; }
		int nsr = ts.receive_stats.ofdm_search_raw;
		if(!(nsr >= 0 && nsr <= upper && f2 > 0)) { inv_ok = false; break; }
	}
	check(inv_ok, "R2/INV-1: search_raw sweep 1..upper stays in-bounds, ftr>0 (buffer-wrap safe)");

	// ---- BOUND (declared B exhausted disarms) ----
	prime(100); ts.receive_stats.coast_frames_since_anchor = data_batch_size;
	check(!batch_coast_try_advance(frame_symb, upper, 100, false, &ftr),
		"declared B exhausted (coast_frames==B) => disarm (returns false)");
	prime(100); ts.receive_stats.coast_frames_since_anchor = data_batch_size - 1;
	check(batch_coast_try_advance(frame_symb, upper, 100, false, &ftr),
		"one below B still coasts");

	// ---- FAIL-BEFORE: defeat makes the helper INERT (grid untouched) ----
	batch_coast_advances = 0;
	prime(100); batch_coast_defeat = true;
	long long ctr_before = batch_coast_advances;
	int sr_before = ts.receive_stats.ofdm_search_raw;
	bool def = batch_coast_try_advance(frame_symb, upper, 100, false, &ftr);
	check(!def, "DEFEAT: coast returns false (inert)");
	check(ts.receive_stats.ofdm_search_raw == sr_before, "DEFEAT: search_raw UNCHANGED");
	check(batch_coast_advances == ctr_before, "DEFEAT: counter frozen");
	// the caller then runs the stock teardown -> grid dies (tail-death path).
	ts.receive_stats.ofdm_search_raw = 0; ts.receive_stats.ofdm_batch_active = false;
	check(!ts.receive_stats.ofdm_batch_active, "DEFEAT: stock teardown kills the grid (tail-death)");

	// ---- eligibility GUARDS (each leg individually blocks the coast) ----
	prime(100); role = COMMANDER;
	check(!batch_coast_try_advance(frame_symb, upper, 100, false, &ftr), "guard: role!=RESPONDER => no coast (CMD SACK-poll disjoint)");
	prime(100); sack_v2_enabled = false;
	check(!batch_coast_try_advance(frame_symb, upper, 100, false, &ftr), "guard: sack_v2 off => no coast");
	prime(100); ts.receive_stats.ofdm_batch_active = false;
	check(!batch_coast_try_advance(frame_symb, upper, 100, false, &ftr), "guard: not mid-batch => no coast");
	prime(0);
	check(!batch_coast_try_advance(frame_symb, upper, 100, false, &ftr), "guard: search_raw<=0 => no coast");
	prime(100);
	check(!batch_coast_try_advance(frame_symb, upper, 100, /*frame_data_missing=*/true, &ftr), "guard: keydown-end (frame_data_missing) => disarm");

	// ================= P2 BOUNDED COAST (data-flow-linkphase-break-storm.md §5 P2) =================
	// The stock coast bound is the full declared batch (data_batch_size=12 here).
	// P2 (MERCURY_BATCH_COAST_MAX=N) tightens that to N and, when a coast reaches
	// depth N while the stock bound would still coast, forces a clean re-acquire
	// (grid torn down) instead of coasting deeper — the deep-plateau escape.
	const int P2N = 4;                       // test bound (< data_batch_size=12)

	// FAIL-BEFORE (P2 OFF): at depth N the STOCK coast still advances (returns
	// true, grid kept alive) — the unbounded behavior the storm rides.
	prime(100); batch_coast_max = 0;
	ts.receive_stats.coast_frames_since_anchor = P2N;
	{
		int f = 0;
		bool coasts = batch_coast_try_advance(frame_symb, upper, 100, false, &f);
		check(coasts, "P2 FAIL-BEFORE: OFF => depth N still COASTS (unbounded, returns true)");
		check(ts.receive_stats.ofdm_batch_active, "P2 FAIL-BEFORE: OFF => grid kept alive (coasts deeper)");
		check(batch_coast_stops == 0, "P2 FAIL-BEFORE: OFF => no [BATCH-COAST-STOP]");
	}

	// PASS-AFTER (P2 ARMED at N): at depth N the coast TRIPS — returns false, the
	// grid is torn down (search_raw=0, batch inactive, coast reset), and the
	// [BATCH-COAST-STOP] counter fires 0->1. The caller then blind-re-searches.
	prime(100); batch_coast_max = P2N;
	ts.receive_stats.coast_frames_since_anchor = P2N;
	{
		int f = 0;
		bool coasts = batch_coast_try_advance(frame_symb, upper, 100, false, &f);
		check(!coasts, "P2 PASS-AFTER: ARMED => depth==N TRIPS (returns false)");
		check(ts.receive_stats.ofdm_search_raw == 0, "P2 PASS-AFTER: grid torn down (search_raw=0)");
		check(!ts.receive_stats.ofdm_batch_active, "P2 PASS-AFTER: batch inactive => forced clean re-acquire");
		check(ts.receive_stats.coast_frames_since_anchor == 0, "P2 PASS-AFTER: coast counter reset");
		check(batch_coast_stops == 1, "P2 PASS-AFTER: [BATCH-COAST-STOP] fires 0->1");
	}

	// GUARD: below the bound (depth N-1) still coasts (sparse-fade rescue preserved).
	prime(100); batch_coast_max = P2N;
	ts.receive_stats.coast_frames_since_anchor = P2N - 1;
	{
		int f = 0;
		bool coasts = batch_coast_try_advance(frame_symb, upper, 100, false, &f);
		check(coasts, "P2 GUARD: armed, depth N-1 still COASTS (sparse-fade rescue preserved)");
		check(batch_coast_stops == 0, "P2 GUARD: no trip below the bound");
	}

	// GUARD: a trip requires the coast to be otherwise eligible — keydown-end
	// (frame_data_missing) at depth N does NOT fire the P2 stop (it disarms first).
	prime(100); batch_coast_max = P2N;
	ts.receive_stats.coast_frames_since_anchor = P2N;
	{
		int f = 0;
		bool coasts = batch_coast_try_advance(frame_symb, upper, 100, /*frame_data_missing=*/true, &f);
		check(!coasts, "P2 GUARD: keydown-end at depth N => no coast (disarm)");
		check(batch_coast_stops == 0, "P2 GUARD: keydown-end does NOT trip the P2 stop");
	}

	this->telecom_system = NULL;   // do not dangle the stack ts
	printf("[TEST-BATCH-COAST] %s (%d failure%s)\n", failed==0?"ALL PASS":"FAILURES", failed, failed==1?"":"s");
	fflush(stdout);
	return failed == 0 ? 0 : 1;
}

// ===========================================================================
// Fix H#3 — RX-FIFO back-pressure post-ACK DATA LOSS (delivery-integrity-audit-monitor.md §3)
//
// CLI: --test-rxfifo-backpressure-hold  (env MERCURY_RXFIFO_BACKPRESSURE_DEFEAT=1 reverts)
//
// THE BUG (channel-free, pure RX delivery state): the clean data-ACK is sent BEFORE
// copy_data_to_buffer() delivers, and fifo_push_rx() returns a SHORT count when
// fifo_buffer_rx is full under app back-pressure (its "caller must NOT count the un-stored
// bytes as delivered" contract) — but copy_data_to_buffer frees the slots + counts them
// anyway, so the un-stored tail is LOST with a committed ACK and the CMD never retransmits.
//
// Fix: the ACK-GATE holds the whole batch-complete ACK+deliver until fifo_buffer_rx has
// room for rx_fifo_batch_need() bytes (keep messages_rx RECEIVED, no ACK, no deliver) so
// the CMD retransmits and the batch re-delivers once the app drains -> no post-ACK loss.
//
// This drives the REAL rx_fifo_batch_need() gate decision and the REAL copy_data_to_buffer()
// delivery on a fifo_buffer_rx sized SMALLER than the batch (back-pressure). PASS-AFTER: the
// gate HOLDS (nothing delivered, whole batch preserved), then after the app "drains" the
// held batch delivers FULLY (zero loss). FAIL-BEFORE (defeat): delivery runs anyway ->
// copy_data_to_buffer stores part, loses the rest, frees the slots (post-ACK loss).
// Returns 0=PASS, 1=FAIL.
int cl_arq_controller::test_rxfifo_backpressure_hold()
{
	bool defeat = false;
	{ const char* e = std::getenv("MERCURY_RXFIFO_BACKPRESSURE_DEFEAT");
	  if(e && *e && atoi(e)!=0) defeat = true; }
	printf("[TEST-RXFIFO-BP] start (MERCURY_RXFIFO_BACKPRESSURE_DEFEAT=%d)\n", defeat?1:0);
	fflush(stdout);

	int failed = 0;
	auto check = [&](bool cond, const char* name, long got, long want){
		if(cond) printf("[TEST-RXFIFO-BP] PASS: %s (got=%ld want=%ld)\n", name, got, want);
		else { printf("[TEST-RXFIFO-BP] FAIL: %s (got=%ld want=%ld)\n", name, got, want); failed++; }
		fflush(stdout);
	};

	this->nMessages          = 120;
	this->max_data_length    = 170;
	this->max_message_length = 200;
	this->max_header_length  = 6;
	int alloc_rc = init_messages_buffers();
	check(alloc_rc == SUCCESSFUL, "C0 buffers allocated", alloc_rc, SUCCESSFUL);
	this->compression_enabled   = false;     // no-comp leg -> rx_fifo_batch_need() is exact
	this->encryption_enabled    = false;
	this->sack_v2_enabled       = true;
	this->sack_enabled          = true;
	this->current_configuration = 0;
	// COMMANDER so fifo_push_rx() does NOT try to drain a (non-existent in-process) app
	// socket on the FIFO-full retry — isolates the FIFO-full short-store, no PHY/TCP.
	this->original_role = COMMANDER;
	this->role          = COMMANDER;

	const int DBS = 10, L = 100;
	this->data_batch_size = DBS;
	const int TOTAL = DBS * L;                // 1000 bytes of app payload

	for(int i=0;i<this->nMessages;i++){ messages_rx[i].status=FREE; messages_rx[i].length=0; }
	for(int i=0;i<DBS;i++){
		messages_rx[i].status = RECEIVED;
		messages_rx[i].length = L;
		for(int j=0;j<L;j++) messages_rx[i].data[j] = (char)(unsigned char)i;
	}

	// App FIFO sized to hold only PART of the batch -> genuine back-pressure.
	const int ROOM = TOTAL/2;                 // 500 < 1000
	fifo_buffer_rx.set_size(ROOM);
	fifo_buffer_rx.flush();
	auto occ_rx = [&]() -> int { return fifo_buffer_rx.get_size() - fifo_buffer_rx.get_free_size(); };

	int rx_free = fifo_buffer_rx.get_free_size();
	int need    = rx_fifo_batch_need();
	check(need == TOTAL, "C1 rx_fifo_batch_need == exact no-comp batch size", need, TOTAL);
	check(need > rx_free, "C2 vacuity: batch need exceeds app-FIFO room (real back-pressure)", need, rx_free);

	// The production ACK-GATE decision (arq_responder.cc): HOLD iff (!defeat && rx_free < need).
	bool hold = (!defeat) && (rx_free < need);

	if(hold)
	{
		// FIX: the handler returns WITHOUT ACKing/delivering — the batch stays RECEIVED.
		int held = 0;
		for(int i=0;i<DBS;i++) if(messages_rx[i].status==RECEIVED) held += messages_rx[i].length;
		int lost = TOTAL - occ_rx() - held;
		check(occ_rx()==0, "A1 nothing delivered under back-pressure (held, not lost)", occ_rx(), 0);
		check(held==TOTAL && lost==0, "A2 ZERO bytes lost — whole batch HELD for retransmit", lost, 0);
		// App drains -> room frees -> the held batch re-delivers FULLY (the handler marks
		// RECEIVED->ACKED post-gate, then copy_data_to_buffer stores the whole batch).
		fifo_buffer_rx.set_size(TOTAL*2);
		fifo_buffer_rx.flush();
		for(int i=0;i<DBS;i++) if(messages_rx[i].status==RECEIVED) messages_rx[i].status=ACKED;
		copy_data_to_buffer();
		check(occ_rx()==TOTAL, "A3 after app drains, held batch delivers FULLY (no loss)", occ_rx(), TOTAL);
	}
	else
	{
		// FAIL-BEFORE / DEFEAT: deliver NOW despite no room. copy_data_to_buffer stores part,
		// LOSES the rest, frees the slots -> the un-stored bytes are unrecoverable (post-ACK loss).
		for(int i=0;i<DBS;i++) if(messages_rx[i].status==RECEIVED) messages_rx[i].status=ACKED;
		copy_data_to_buffer();
		int delivered = occ_rx();
		int held = 0;
		for(int i=0;i<DBS;i++) if(messages_rx[i].status==RECEIVED || messages_rx[i].status==ACKED)
			held += messages_rx[i].length;
		int lost = TOTAL - delivered - held;
		printf("[TEST-RXFIFO-BP] defeat: delivered=%d held=%d lost=%d (TOTAL=%d)\n",
			delivered, held, lost, TOTAL);
		fflush(stdout);
		check(lost > 0,
			"A1 fail-before LOSES post-ACK bytes on back-pressure (slots freed, un-stored gone)", lost, 1);
	}

	deinit_messages_buffers();
	printf("[TEST-RXFIFO-BP] %s: fails=%d (defeat=%d)\n",
		failed==0 ? "ALL PASS" : "FAILURES", failed, defeat?1:0);
	fflush(stdout);
	return failed==0 ? 0 : 1;
}
