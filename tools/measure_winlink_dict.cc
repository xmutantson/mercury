/*
 * measure_winlink_dict — head-to-head real-traffic compression measurement for the
 * Winlink-boilerplate priming dictionary.
 *
 * For each representative Winlink small message (B2F-unrolled plaintext as the modem
 * actually sees it, INCLUDING the RMS Express form XML attachment scaffolding):
 *   - Mercury COLD   : cl_compressor, dict priming OFF (the pre-dict baseline)
 *   - Mercury PRIMED : cl_compressor, dict priming ON  (the v1 universal dict)
 *   - VARA-equiv     : lzhuf_encode_buffer (the FBB/B2F LZHUF VARA uses), which is
 *                      structurally COLD per message (no dict inject, fresh window).
 *
 * Reports per-message ratios and the AGGREGATE x-VARA edge = (sum VARA wire) /
 * (sum Mercury wire). >1.0 means Mercury's total wire is SMALLER than VARA's = BEAT.
 * Run twice (before/after the dict enrichment) to read the delta.
 *
 * BUILD-TIME tool. Not linked into firmware.
 */
#include "compression/mercury_compress.h"
#include "compression/winlink_dict.h"
#include "compression/lzhuf_buffer.h"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>

// GENUINE RMS Express .mime messages — the exact B2F-unrolled wire bytes the modem
// compresses (extracted from C:/RMS Express/KG7VSN/Messages, CRLF-faithful). These
// carry the real text/plain rendered form body AND the base64 form-XML attachment,
// exactly as transmitted. The synthetic corpus below is kept for continuity.
#include "winlink_real_corpus.inc"

// One Mercury batch through a fresh (cold or primed) streaming context.
static int mercury_wire(const std::string& m, bool prime)
{
	cl_compressor c; c.set_dict_priming(prime); c.init(); c.streaming_enable();
	static char out[262144];
	int w = c.compress_block(m.data(), (int)m.size(), out, (int)sizeof(out));
	// Verify bit-exact round-trip through a matching peer (integrity gate).
	int rt_ok = 0;
	if (w > 0) {
		cl_compressor rx; rx.set_dict_priming(prime); rx.init(); rx.streaming_enable();
		static char rt[262144];
		int d = rx.decompress_block(out, w, rt, (int)sizeof(rt));
		rt_ok = (d == (int)m.size() && memcmp(rt, m.data(), m.size()) == 0);
		rx.deinit();
	}
	c.deinit();
	return rt_ok ? w : -1;
}

static int vara_wire(const std::string& m)
{
	static uint8_t out[262144]; size_t ol = 0;
	if (lzhuf_encode_buffer((const uint8_t*)m.data(), m.size(), out, sizeof(out), &ol) != 0)
		return (int)m.size(); // VARA falls back to raw when LZHUF doesn't help
	return ol < m.size() ? (int)ol : (int)m.size();
}

struct Sample { const char* name; std::string body; };

// Measure one corpus: per-sample cold/primed/vara + aggregate x-VARA edge. Returns
// 0 if every batch round-tripped bit-exact, else 1.
static int measure_corpus(const char* title, std::vector<Sample>& corpus)
{
	printf("=== %s (dict v%d, raw=%u compressed=%u) ===\n",
		title, WINLINK_DICT_VERSION, WINLINK_DICT_RAW_LEN, WINLINK_DICT_COMPRESSED_LEN);
	printf("%-18s %6s | merc-cold       merc-primed     | vara(lzhuf)\n", "sample", "orig");

	long sum_orig = 0, sum_cold = 0, sum_primed = 0, sum_vara = 0;
	int any_fail = 0;
	for (auto& s : corpus) {
		int orig   = (int)s.body.size();
		int cold   = mercury_wire(s.body, false);
		int primed = mercury_wire(s.body, true);
		int vara   = vara_wire(s.body);
		if (cold <= 0 || primed <= 0) { any_fail = 1; }
		double cr = cold   > 0 ? (double)orig / cold   : 0;
		double pr = primed > 0 ? (double)orig / primed : 0;
		double vr = vara   > 0 ? (double)orig / vara   : 0;
		printf("%-18s %6d | %5d (%.2fx)   %5d (%.2fx)   | %5d (%.2fx)\n",
			s.name, orig, cold, cr, primed, pr, vara, vr);
		sum_orig += orig; sum_cold += cold; sum_primed += primed; sum_vara += vara;
	}
	double edge_cold   = sum_cold   > 0 ? (double)sum_vara / sum_cold   : 0;
	double edge_primed = sum_primed > 0 ? (double)sum_vara / sum_primed : 0;
	printf("---\n");
	printf("aggregate orig=%ld  merc_cold=%ld  merc_primed=%ld  vara=%ld\n",
		sum_orig, sum_cold, sum_primed, sum_vara);
	printf("aggregate x-VARA edge: cold=%.3f  primed=%.3f  (>1.0 = Mercury beats VARA wire)\n",
		edge_cold, edge_primed);
	printf("round-trip integrity: %s\n\n", any_fail ? "FAIL (a batch did not round-trip)" : "OK (all bit-exact)");
	return any_fail ? 1 : 0;
}

int main()
{
	// --- GENUINE real-traffic corpus (actual RMS Express .mime, CRLF-faithful) ---
	std::vector<Sample> real_corpus;
	real_corpus.push_back({"ics213-real",   std::string(MSG_ICS213_REAL,  sizeof(MSG_ICS213_REAL)-1)});
	real_corpus.push_back({"ics213-real2",  std::string(MSG_ICS213_REAL2, sizeof(MSG_ICS213_REAL2)-1)});
	real_corpus.push_back({"aar-real",      std::string(MSG_AAR_REAL,     sizeof(MSG_AAR_REAL)-1)});
	real_corpus.push_back({"plain-real",    std::string(MSG_PLAIN_REAL,   sizeof(MSG_PLAIN_REAL)-1)});

	std::vector<Sample> corpus;

	// --- 1. Short Winlink check-in (text body, no form) ---
	corpus.push_back({"short-checkin",
		"Mid: 9QZ2WB7HKLMN\nBody: 214\nContent-Transfer-Encoding: 8bit\n"
		"Content-Type: text/plain; charset=ISO-8859-1\nDate: 2026/06/12 18:02\n"
		"From: N0CALL\nMbo: N0CALL\nSubject: Check in - net control\nTo: SECTION-NET\n\n"
		"Net control checking in for the Tuesday evening traffic net. Conditions\n"
		"fair on 80 meters, some QSB. Two pieces of traffic to pass for the\n"
		"northern district. Standing by. 73 de N0CALL.\n"});

	// --- 2. ICS-213 general message form (body + XML attachment scaffold) ---
	corpus.push_back({"ics213-form",
		"Mid: 7R3PLMNB45QZ\nBody: 402\nContent-Transfer-Encoding: 8bit\n"
		"Content-Type: text/plain; charset=ISO-8859-1\nDate: 2026/06/12 09:15\n"
		"From: AC4EM\nMbo: AC4EM\nSubject: //WL2K ICS-213 General Message - Resource Request\n"
		"To: EOC-COUNTY\n\n"
		"<?xml version=\"1.0\"?>\n<RMS_Express_Form>\n<form_parameters>\n"
		"<xml_file_name>RMS_Express_Form_ICS213_Initial_Viewer.xml</xml_file_name>\n"
		"<form_version>1.0</form_version>\n</form_parameters>\n<variables>\n"
		"1. Incident Name: Hurricane Response Exercise 2026\n"
		"2. To (Name/Position): County EOC Logistics Section Chief\n"
		"3. From (Name/Position): Shelter 4 Communications Unit Leader\n"
		"4. Subject: Resource Request - Cots and Water\n5. Date: 06/12/2026\n6. Time: 0915\n"
		"7. Message:\nRequest the following resources be delivered to Shelter 4 no later\n"
		"than 1400 today: 150 folding cots, 40 cases bottled water, 6 light towers.\n"
		"Current shelter population is 312 and rising.\n"
		"8. Approved by: J. Martinez\nPosition/Title: Shelter Manager\n"
		"</variables>\n</RMS_Express_Form>\n"});

	// --- 3. Winlink Check-in form (the WNLNK form, XML attachment scaffold) ---
	corpus.push_back({"checkin-form",
		"Mid: K2L8MNPQ9XZ4\nBody: 318\nContent-Transfer-Encoding: 8bit\n"
		"Content-Type: text/plain; charset=ISO-8859-1\nDate: 2026/06/12 14:40\n"
		"From: KD7ABC\nMbo: KD7ABC\nSubject: Winlink Check-In\nTo: WL2KNET\n\n"
		"<?xml version=\"1.0\"?>\n<RMS_Express_Form>\n<form_parameters>\n"
		"<xml_file_name>RMS_Express_Form_Winlink_Check_In_Viewer.xml</xml_file_name>\n"
		"</form_parameters>\n<variables>\n"
		"Status: Check In\nGroup/Net Name: County ARES Net\nBand: 80m\nMode: VARA HF\n"
		"Power: 100\nComments: Check in from home QTH, all stations copied.\n"
		"Call Sign: KD7ABC\nName: Robert\nLocation: Springfield\nCounty: Greene\nState: MO\n"
		"</variables>\n</RMS_Express_Form>\n"});

	// --- 4. Position report ---
	corpus.push_back({"position-report",
		"Mid: P9R4STUV1WX7\nBody: 188\nContent-Transfer-Encoding: 8bit\n"
		"Content-Type: text/plain; charset=ISO-8859-1\nDate: 2026/06/12 11:05\n"
		"From: W6XYZ\nMbo: W6XYZ\nSubject: Position Report\nTo: QTH\n\n"
		"Position report\nLatitude: 44-24.02N\nLongitude: 026-05.06E\n"
		"Grid square: KN34bj\nComment: Mobile, en route to staging area.\n"});

	// --- 5. Plain email (free prose, no form — the inert / parity case) ---
	corpus.push_back({"plain-email",
		"Mid: E3M6FGHI8JK2\nBody: 246\nContent-Transfer-Encoding: 8bit\n"
		"Content-Type: text/plain; charset=ISO-8859-1\nDate: 2026/06/12 20:11\n"
		"From: VE3QRS\nMbo: VE3QRS\nSubject: Re: weekend plans\nTo: VE3TUV\n\n"
		"Thanks for the note. I should be free Saturday afternoon to help with the\n"
		"antenna project. Bring the analyzer if you have it; mine is acting up. I will\n"
		"grab coffee on the way. Let me know what time works. 73.\n"});

	// x-VARA edge = VARA total wire / Mercury total wire. >1 => Mercury smaller => beat.
	int f1 = measure_corpus("REAL RMS Express .mime traffic", real_corpus);
	int f2 = measure_corpus("synthetic corpus (legacy continuity)", corpus);
	return (f1 || f2) ? 1 : 0;
}
