/*
 * Mercury: A configurable open-source software-defined modem.
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
 */

// Library tests for the sounding probe / EESM election (no modem state).
// Entry: mercury --test-eesm-probe (also runs the modem-chain loopback).

#include "physical_layer/eesm_probe.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>

namespace eesm_probe {

typedef std::complex<double> cplx;

namespace {

struct t_rng {
	uint64_t s;
	bool have; double spare;
	explicit t_rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ULL + 1), have(false), spare(0.0) {}
	uint64_t u64() { s += 0x9E3779B97F4A7C15ULL; uint64_t z = s; z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL; z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL; return z ^ (z >> 31); }
	double uni() { return (double)(u64() >> 11) * (1.0 / 9007199254740992.0); }
	double gauss() {
		if(have) { have = false; return spare; }
		double u1 = uni(), u2 = uni(); if(u1 < 1e-300) u1 = 1e-300;
		const double r = std::sqrt(-2.0 * std::log(u1));
		spare = r * std::sin(2.0 * M_PI * u2); have = true;
		return r * std::cos(2.0 * M_PI * u2);
	}
	cplx cn(double var) { const double s_ = std::sqrt(var / 2.0); return cplx(gauss() * s_, gauss() * s_); }
};

int g_fail = 0;
#define EP_CHECK(cond, ...) do { if(cond) { std::printf("  PASS "); } else { std::printf("  FAIL "); g_fail++; } std::printf(__VA_ARGS__); std::printf("\n"); } while(0)

// Synthetic demodulated grid: Y = H_k(s) X + W, noise power N per cell.
static void synth_grid(const st_geometry& g, const std::vector<cplx>& Hks /*S*Nc*/, double N,
	t_rng& rng, std::vector<cplx>& Y)
{
	Y.assign((size_t)g.n_symb * g.Nc, cplx(0.0, 0.0));
	std::vector<cplx> X((size_t)g.Nc);
	for(int s = 0; s < g.n_symb; s++)
	{
		symbol_cells(g, s, X.data());
		for(int k = 0; k < g.Nc; k++)
			Y[(size_t)s * g.Nc + k] = Hks[(size_t)s * g.Nc + k] * X[(size_t)k] + rng.cn(N);
	}
}

static void t_geometry()
{
	std::printf("[EESM-TEST] geometry + waveform\n");
	st_geometry g = wb_geometry();
	std::vector<int> seen(256, 0);
	bool distinct = true;
	for(int k = 0; k < g.Nc; k++) { int b = bin_of_carrier(g, k); if(b < 0 || b >= g.Nfft || seen[(size_t)b]++) distinct = false; }
	EP_CHECK(distinct && !seen[0], "carrier->bin map distinct, DC bin unused");
	int n0 = 0, n1 = 0; bool unit = true; double p0 = 0.0, p1 = 0.0;
	std::vector<cplx> c((size_t)g.Nc);
	symbol_cells(g, 0, c.data());
	for(int k = 0; k < g.Nc; k++) { if(std::abs(c[(size_t)k]) > 0) { n0++; p0 += std::norm(c[(size_t)k]); } }
	symbol_cells(g, 1, c.data());
	for(int k = 0; k < g.Nc; k++) { if(std::abs(c[(size_t)k]) > 0) { n1++; p1 += std::norm(c[(size_t)k]); if(std::fabs(std::abs(c[(size_t)k]) - std::abs(c[(size_t)(k)])) > 1e-12) unit = false; } }
	EP_CHECK(n0 + n1 == g.Nc && n0 > 0 && n1 > 0, "comb-2 covers every carrier once per symbol pair (%d + %d)", n0, n1);
	EP_CHECK(std::fabs(p0 - g.Nc) < 1e-9 && std::fabs(p1 - g.Nc) < 1e-9 && unit,
		"symbol power == Nc unit cells (%.6f, %.6f)", p0, p1);
	std::vector<cplx> bb((size_t)probe_samples(g));
	const int n = build_baseband(g, bb.data(), (int)bb.size());
	double pk = 0.0, rms = 0.0;
	for(int i = 0; i < n; i++) { pk = std::max(pk, std::abs(bb[(size_t)i])); rms += std::norm(bb[(size_t)i]); }
	rms = std::sqrt(rms / n);
	// half-symbol repetition: even-bin symbol x[m+N/2] = x[m], odd-bin = -x[m]
	double rep_err = 0.0;
	for(int s = 0; s < 2; s++)
		for(int m = 0; m < g.Nfft / 2; m++)
		{
			const cplx a = bb[(size_t)s * symbol_samples(g) + g.Ngi + m];
			const cplx b = bb[(size_t)s * symbol_samples(g) + g.Ngi + m + g.Nfft / 2];
			rep_err = std::max(rep_err, std::abs(b - ((s & 1) ? -a : a)));
		}
	EP_CHECK(n == probe_samples(g) && rep_err < 1e-12, "baseband %d samples, half-symbol repetition err %.2e", n, rep_err);
	std::printf("  INFO complex-baseband PAPR %.2f dB (before interpolation and TX filters)\n", 20.0 * std::log10(pk / rms));
}

static void t_eesm_math()
{
	std::printf("[EESM-TEST] EESM mapping\n");
	double f[50];
	for(int k = 0; k < 50; k++) f[k] = 10.0;
	EP_CHECK(std::fabs(eesm_db(f, 50, 3.0) - 10.0) < 1e-9, "flat vector -> eesm == its value");
	f[7] = 0.0;
	const double a = eesm_db(f, 50, 3.0), b = eesm_db(f, 50, 300.0);
	EP_CHECK(a < 10.0 && b > a, "one null lowers eesm, larger beta approaches the mean (%.3f %.3f dB)", 10*std::log10(std::pow(10,a/10)), b);
	for(int k = 0; k < 50; k++) f[k] = 1e7 + k;
	EP_CHECK(std::fabs(eesm_db(f, 50, 0.1) - 70.0) < 0.01, "large SINR does not underflow (%.4f dB)", eesm_db(f, 50, 0.1));
}

static void t_estimator_flat()
{
	std::printf("[EESM-TEST] estimator on synthetic flat grids (unbiased, sigma calibrated)\n");
	st_geometry g = wb_geometry();
	const double snrs[] = { -10.0, 0.0, 10.0, 20.0, 30.0 };
	for(double snr : snrs)
	{
		t_rng rng(1000 + (uint64_t)(snr + 50));
		const int T = 300;
		double sum_lin = 0.0, s2 = 0.0, sig = 0.0; int nok = 0;
		std::vector<cplx> H((size_t)g.n_symb * g.Nc), Y;
		for(int t = 0; t < T; t++)
		{
			const cplx h0 = std::polar(std::sqrt(std::pow(10.0, snr / 10.0)), rng.uni() * 2 * M_PI);
			std::fill(H.begin(), H.end(), h0);
			synth_grid(g, H, 1.0, rng, Y);
			st_measurement m;
			if(!estimate_from_grid(g, Y.data(), &m)) continue;
			double gm = 0.0; for(int k = 0; k < g.Nc; k++) gm += m.sinr_lin[k]; gm /= g.Nc;
			const double r = gm / std::pow(10.0, snr / 10.0);
			sum_lin += r; s2 += r * r; nok++;
			sig += m.sigma_mean_db;
		}
		const double mean = sum_lin / nok, sd = std::sqrt(std::max(0.0, s2 / nok - mean * mean));
		const double emp_db = 10.0 / std::log(10.0) * sd / mean, pred_db = sig / nok;
		const double bias_db = 10.0 * std::log10(mean);
		// unbiased within 3 standard errors of the mean (plus 0.05 dB numerical slack);
		// the predicted sigma must match the empirical spread within +-35%
		const double se_db = emp_db / std::sqrt((double)nok);
		EP_CHECK(nok == T && std::fabs(bias_db) < 3.0 * se_db + 0.05 && emp_db < 1.35 * pred_db && emp_db > 0.65 * pred_db,
			"flat %+5.1f dB: bias %+.3f dB (se %.3f)  sd %.3f dB  predicted %.3f dB  (n=%d)", snr, bias_db, se_db, emp_db, pred_db, nok);
	}
}

static void t_estimator_selective()
{
	std::printf("[EESM-TEST] estimator on a static two-path channel (per-carrier truth)\n");
	st_geometry g = wb_geometry();
	const double snr = 20.0, S = std::pow(10.0, snr / 10.0);
	t_rng rng(77);
	const double betas[] = { 1.0, 3.0, 6.245 };
	for(double beta : betas)
	{
		double e_sum = 0.0, e2 = 0.0; int n = 0;
		for(int t = 0; t < 200; t++)
		{
			const double tau = 1e-3 + 1e-3 * rng.uni();          // 1..2 ms echo
			const cplx g1 = std::polar(0.9, rng.uni() * 2 * M_PI);
			std::vector<cplx> H((size_t)g.n_symb * g.Nc), Y;
			std::vector<double> gt((size_t)g.Nc);
			for(int k = 0; k < g.Nc; k++)
			{
				const int b = bin_of_carrier(g, k);
				const int q = (b >= g.Nfft / 2) ? b - g.Nfft : b;
				const double f = q * g.fs / g.Nfft;
				const cplx h = (1.0 + g1 * std::polar(1.0, -2 * M_PI * f * tau)) / std::sqrt(1.81) * std::sqrt(S);
				gt[(size_t)k] = std::norm(h);
				for(int s = 0; s < g.n_symb; s++) H[(size_t)s * g.Nc + k] = h;
			}
			synth_grid(g, H, 1.0, rng, Y);
			st_measurement m;
			if(!estimate_from_grid(g, Y.data(), &m)) continue;
			const double err = eesm_db(m.sinr_lin, g.Nc, beta) - eesm_db(gt.data(), g.Nc, beta);
			e_sum += err; e2 += err * err; n++;
		}
		const double mean = e_sum / n, sd = std::sqrt(std::max(0.0, e2 / n - mean * mean));
		EP_CHECK(n == 200 && std::fabs(mean) < 0.35 && sd < 0.6,
			"two-path 20 dB beta %.2f: eesm error mean %+.3f dB sd %.3f dB (n=%d)", beta, mean, sd, n);
	}
}

static void t_time_selectivity()
{
	std::printf("[EESM-TEST] time selectivity: common rotation removed, independent variation measured\n");
	st_geometry g = wb_geometry();
	t_rng rng(5);
	const double S = 1e3;       // 30 dB
	std::vector<cplx> H((size_t)g.n_symb * g.Nc), Y;
	// (a) common 3 Hz rotation only
	for(int s = 0; s < g.n_symb; s++)
		for(int k = 0; k < g.Nc; k++)
			H[(size_t)s * g.Nc + k] = std::sqrt(S) * std::polar(1.0, 2 * M_PI * 3.0 * s * symbol_samples(g) / g.fs);
	synth_grid(g, H, 1.0, rng, Y);
	st_measurement m;
	estimate_from_grid(g, Y.data(), &m);
	EP_CHECK(m.time_var_frac < 0.002 && std::fabs(m.cfo_hz - 3.0) < 0.05,
		"common 3 Hz rotation: cfo %.3f Hz, time_var_frac %.5f", m.cfo_hz, m.time_var_frac);
	// (b) independent per-cell variation with power fraction 0.05 of the channel
	for(int s = 0; s < g.n_symb; s++)
		for(int k = 0; k < g.Nc; k++)
			H[(size_t)s * g.Nc + k] = std::sqrt(S) * (cplx(std::sqrt(0.95), 0.0) + rng.cn(0.05));
	synth_grid(g, H, 1.0, rng, Y);
	estimate_from_grid(g, Y.data(), &m);
	EP_CHECK(m.time_var_frac > 0.035 && m.time_var_frac < 0.07,
		"independent 5%% variation: time_var_frac %.4f", m.time_var_frac);
}

static void t_baseband_path()
{
	std::printf("[EESM-TEST] baseband receive path: timing, frequency, echo, noise\n");
	st_geometry g = wb_geometry();
	const int np = probe_samples(g);
	const int lead = 700, tail = 700;
	const double cfo = 17.3, snr_db = 15.0;
	const int echo = 14;              // samples (1.17 ms) inside the prefix
	const cplx g1 = std::polar(0.7, 1.1);
	std::vector<cplx> ref((size_t)np);
	build_baseband(g, ref.data(), np);
	double sp = 0.0; for(int i = 0; i < np; i++) sp += std::norm(ref[(size_t)i]); sp /= np;
	// per-sample noise so that the per-carrier SNR of one unit data cell is snr_db:
	// a unit cell contributes |x|^2 = 1/Nfft^2 per sample per carrier after the 1/Nfft
	// IFFT; per-carrier noise after an Nfft FFT of per-sample variance v is Nfft*v while
	// the cell comes back as amplitude 1 -> SNR = 1/(Nfft*v).
	const double v = 1.0 / (g.Nfft * std::pow(10.0, snr_db / 10.0));
	const int n = lead + np + tail;
	std::vector<cplx> bb((size_t)n);
	t_rng rng(99);
	for(int i = 0; i < n; i++)
	{
		cplx x(0.0, 0.0);
		const int j = i - lead;
		if(j >= 0 && j < np) x += ref[(size_t)j];
		if(j - echo >= 0 && j - echo < np) x += g1 * ref[(size_t)(j - echo)];
		x *= std::polar(1.0 / std::sqrt(1.0 + std::norm(g1)), 2 * M_PI * cfo * i / g.fs);
		bb[(size_t)i] = x + rng.cn(v);
	}
	st_measurement m;
	const bool ok = estimate_from_baseband(g, bb.data(), n, 0, lead + tail, &m);
	double gt[kMaxNc];
	for(int k = 0; k < g.Nc; k++)
	{
		const int b = bin_of_carrier(g, k);
		const cplx h = (1.0 + g1 * std::polar(1.0, -2 * M_PI * b * echo / (double)g.Nfft)) / std::sqrt(1.0 + std::norm(g1));
		gt[k] = std::norm(h) * std::pow(10.0, snr_db / 10.0);
	}
	double gm = 0.0; for(int k = 0; k < g.Nc; k++) gm += gt[k]; gm /= g.Nc;
	EP_CHECK(ok && m.timing_offset >= lead - 2 && m.timing_offset <= lead + echo + 2,
		"timing offset %d (probe at %d, echo +%d), metric %.3f", m.timing_offset, lead, echo, m.timing_metric);
	EP_CHECK(ok && std::fabs(m.cfo_hz - cfo) < 0.3, "frequency offset %.3f Hz (true %.1f)", m.cfo_hz, cfo);
	EP_CHECK(ok && std::fabs(m.mean_snr_db - 10 * std::log10(gm)) < 3.0 * m.sigma_mean_db + 0.1,
		"band SNR %.2f dB vs truth %.2f dB (sigma %.2f)", m.mean_snr_db, 10 * std::log10(gm), m.sigma_mean_db);
	const double e3 = eesm_db(m.sinr_lin, g.Nc, 3.0) - eesm_db(gt, g.Nc, 3.0);
	EP_CHECK(ok && std::fabs(e3) < 0.8, "eesm(beta 3) error %+.3f dB", e3);
	// timing search must not lock two symbols early/late (distinct Chu roots per pair)
	// expected metric: the matched filter collects the strongest path coherently, so
	// ~ (|g0|^2/(|g0|^2+|g1|^2)) * S/(S+N) with per-sample SNR = carrier SNR * Nc/Nfft
	const double snr_t = std::pow(10.0, snr_db / 10.0) * g.Nc / g.Nfft;
	const double expm = (1.0 / (1.0 + std::norm(g1))) * snr_t / (1.0 + snr_t);
	EP_CHECK(ok && m.timing_metric > 0.8 * expm, "detection metric %.3f (expected ~%.3f)", m.timing_metric, expm);
}

static st_measurement flat_measurement(double snr_db, double var_scale)
{
	st_measurement m;
	std::memset(&m, 0, sizeof(m));
	m.valid = true; m.Nc = 50;
	const double gl = std::pow(10.0, snr_db / 10.0);
	for(int k = 0; k < 50; k++) { m.sinr_lin[k] = gl; m.var_sinr[k] = var_scale * gl * gl; m.noise[k] = 1.0; }
	m.mean_snr_db = snr_db;
	m.sigma_mean_db = 0.01;
	m.noise_rel_var = 1e-6;
	return m;
}

static void t_decide()
{
	std::printf("[EESM-TEST] election\n");
	st_policy p = default_policy();
	EP_CHECK(std::fabs(p.guard.max_time_var_frac - 0.000255) < 1e-12 && p.guard.safe_top_cfg == 15,
		"registered cfg16 guard split is installed (tvar %.6f, safe cfg%d)",
		p.guard.max_time_var_frac, p.guard.safe_top_cfg);
	{
		const st_acq_row* a = acquisition_table();
		bool indexed = std::strcmp(acquisition_table_build(), "310fad44ff") == 0;
		for(int c = 0; c < kNumOfdmCfg; c++)
			indexed = indexed && a[c].cfg == c && p.acq_floor_db[c] == a[c].floor_probe_db;
		EP_CHECK(indexed && a[0].measured && a[7].measured && a[15].measured && !a[16].measured,
			"versioned frame-0 acquisition ladder is indexed and marks measured/extrapolated rows");
		EP_CHECK(std::fabs(a[0].floor_probe_db - 12.39) < 1e-12 &&
			std::fabs(a[15].floor_probe_db - 20.39) < 1e-12,
			"receiver snr3k floors include the measured +6.39 dB probe-axis conversion");
	}
	{
		// acquisition floor: a config whose decode threshold is met but whose frame-0
		// floor is not must not be elected
		const st_cfg_row* t = cfg_table();
		st_measurement m = flat_measurement(t[15].knee01_db + t[15].awgn_offset_db + 0.3, 1e-8);
		st_decision d = decide(m, p);
		const bool floor_on = p.use_acq_floor;
		EP_CHECK(!floor_on || (d.rung < 15 && m.mean_snr_db < p.acq_floor_db[15]),
			"acquisition floor holds cfg15 back below its frame-0 floor (rung %d, floor %.2f, snr %.2f)",
			d.rung, p.acq_floor_db[15], m.mean_snr_db);
	}
	p.use_acq_floor = false;   // the remaining cases test the EESM election itself
	const st_cfg_row* t = cfg_table();
	const int targets[] = { 5, 10, 13, 15, 16 };
	for(int c : targets)
	{
		st_measurement m = flat_measurement(t[c].knee01_db + t[c].awgn_offset_db + 0.3, 1e-8);
		st_decision d = decide(m, p);
		// highest config whose threshold is <= the target's (ties elect the higher index)
		int want = -1;
		for(int j = 0; j <= p.top_cfg; j++)
			if(t[j].knee01_db + t[j].awgn_offset_db <= t[c].knee01_db + t[c].awgn_offset_db + 0.3 - 1e-9) want = j;
		EP_CHECK(d.valid && d.open_wb && d.rung == want, "flat at thr(%d)+0.3 dB elects %d (want %d)", c, d.rung, want);
	}
	{
		st_measurement m = flat_measurement(t[16].knee01_db + 0.2, 0.05);   // big uncertainty
		st_decision d = decide(m, p);
		EP_CHECK(d.valid && d.rung < 16 && d.margin_db > 0.2, "uncertainty margin keeps a marginal cfg16 out (rung %d, margin %.2f dB)", d.rung, d.margin_db);
	}
	{
		st_measurement m = flat_measurement(40.0, 1e-8);
		st_decision d = decide(m, p);
		EP_CHECK(d.rung == 16 && d.cap_cfg == 16, "40 dB flat elects the top rung 16 (cfg17 not above WB_CONFIG_MAX)");
		st_policy pg = p; pg.guard.max_time_var_frac = 0.01; m.time_var_frac = 0.02;
		d = decide(m, pg);
		EP_CHECK(d.rung == 15 && d.cap_cfg == 15 && d.time_selective, "time-selective channel caps at cfg15 (rung %d)", d.rung);
		m.time_var_frac = 0.0; pg = p; pg.guard.max_freq_sel_db = 1.0; m.freq_sel_db = 2.0;
		d = decide(m, pg);
		EP_CHECK(d.rung == 15 && d.freq_selective, "frequency-selective channel caps at cfg15 (rung %d)", d.rung);
	}
	{
		st_measurement m = flat_measurement(-20.0, 1e-8);
		st_decision d = decide(m, p);
		EP_CHECK(d.valid && !d.open_wb && d.rung == kRobustBase, "below the cfg0 knee -> ROBUST_0 narrowband entry");
		m = flat_measurement(30.0, 1e-8);
		st_policy pn = p; pn.wb_possible = false;
		d = decide(m, pn);
		EP_CHECK(d.valid && !d.open_wb && d.rung == kRobustBase, "WB not possible -> ROBUST_0");
		m.valid = false;
		d = decide(m, p);
		EP_CHECK(!d.valid && d.rung == kRungNone, "no measurement -> no evidence");
	}
}

static void t_reply()
{
	std::printf("[EESM-TEST] reply codec\n");
	bool all = true;
	for(int r = -1; r < 22; r++)
	{
		st_decision d; std::memset(&d, 0, sizeof(d));
		d.valid = (r >= 0);
		d.rung = (r < 0) ? kRungNone : (r < 18 ? r : kRobustBase + r - 18);
		d.cap_cfg = 15; d.eff_at_rung_db = -3.5 + r; d.margin_db = 0.5;
		d.time_selective = (r & 1); d.freq_selective = (r & 2);
		uint32_t w = encode_reply(d);
		unsigned char b[3]; reply_to_bytes(w, b);
		st_reply q;
		const bool ok = decode_reply(reply_from_bytes(b), &q);
		if(!ok || q.valid != d.valid || (d.valid && q.rung != d.rung) || (d.valid && q.cap_cfg != 15)
		   || std::fabs(q.eff_db - d.eff_at_rung_db) > 0.25 + 1e-9 || std::fabs(q.margin_db - 0.5) > 1e-9
		   || q.time_selective != d.time_selective || q.freq_selective != d.freq_selective || w > 0xFFFFFF)
			all = false;
	}
	EP_CHECK(all, "24-bit round trip over every rung code");
	st_decision d; std::memset(&d, 0, sizeof(d));
	d.valid = true; d.rung = 16; d.cap_cfg = 16; d.eff_at_rung_db = 99.0; d.margin_db = 9.0;
	st_reply q; decode_reply(encode_reply(d), &q);
	EP_CHECK(std::fabs(q.eff_db - 43.5) < 1e-9 && std::fabs(q.margin_db - 1.75) < 1e-9, "saturation (eff %.1f, margin %.2f)", q.eff_db, q.margin_db);
	EP_CHECK(!decode_reply(0x1000000u, &q) && !decode_reply(encode_reply(d) | 1u, &q), "malformed words rejected");
}

} // namespace

int run_unit_tests()
{
	g_fail = 0;
	t_geometry();
	t_eesm_math();
	t_estimator_flat();
	t_estimator_selective();
	t_time_selectivity();
	t_baseband_path();
	t_decide();
	t_reply();
	std::printf("[EESM-TEST] library: %d failure(s)\n", g_fail);
	return g_fail;
}

} // namespace eesm_probe
