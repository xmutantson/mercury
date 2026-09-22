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

// Link-quality sounding probe + per-carrier SINR estimator + EESM config election.
//
// Purpose: before any data flows, the commander keys one short wideband sounding
// burst; the responder measures the SINR of every OFDM carrier from it, compresses
// the vector per configuration with the exponential effective-SINR mapping (EESM)
// and elects the highest configuration whose effective SINR clears that
// configuration's AWGN knee at a one-sided 95% lower confidence bound. The result
// travels back in 24 bits. Nothing here keys the radio or touches the ARQ state
// machine: the handshake owns the exchange, the gearshift consumes the decision.
//
// Waveform (no new waveform family): the WB OFDM numerology (Nfft 256, Ngi 54,
// Nc 50, 12 kHz baseband) with n_symb sounding symbols on a comb-2 lattice. Symbol
// s carries a unit-modulus Chu sequence on the carriers whose FFT bin parity equals
// s mod 2 and nothing on the others, scaled so the symbol power equals a data symbol
// (Nc unit-energy cells). This is the structure of the WB preamble (an every-4th-bin
// comb whose time signal repeats inside the symbol) and of the LTE uplink sounding
// reference signal (3GPP TS 36.211 5.5.3: Zadoff-Chu on a transmission comb). The
// even-bin symbols repeat every Nfft/2 samples and the odd-bin symbols repeat with a
// sign flip, which gives a Moose/Schmidl-Cox frequency estimate over +-fs/(2*Nfft/2).
// Every carrier is sounded in n_symb/2 symbols and silent in the other n_symb/2: the
// silent cells are the noise reference measured inside the probe itself.
//
// Estimator basis:
//  - per-carrier channel: least squares on the known cells, averaged over the active
//    symbols after removing a common frequency slope (pilot-aided SINR estimation,
//    Ozdemir & Arslan, IEEE Comm. Surveys 2007 IV; Boumard, "Novel noise variance and
//    SNR estimation algorithm for wireless MIMO OFDM systems", GLOBECOM 2003);
//  - noise: the silent cells (null-subcarrier noise estimation, same survey), pooled
//    over the band unless a chi-square homogeneity test rejects a white floor;
//  - optional delay-domain smoothing with even-mirror band-edge extension (Edfors et
//    al. VTC 1995; Li et al. VTC 2006), used only when the energy it removes is
//    consistent with noise alone (so it never cuts real channel structure);
//  - EESM: gamma_eff = -beta * ln(mean_k exp(-gamma_k/beta)) (Brueninghaus et al.,
//    "Link performance models for system level simulations of broadband radio access
//    systems", PIMRC 2005; 3GPP TR 25.892 annex), beta and the knee ladder from the
//    in-tree passband BER calibration (cfg0-17, AWGN + 2-path Gaussian-Doppler).

#ifndef INC_EESM_PROBE_H_
#define INC_EESM_PROBE_H_

#include <complex>
#include <cstdint>

namespace eesm_probe {

static const int kMaxNc = 64;
static const int kMaxSymb = 16;
static const int kNumOfdmCfg = 18;        // CONFIG_0 .. CONFIG_17
static const int kRungNone = -1;
static const int kRobustBase = 100;       // ROBUST_0 .. ROBUST_3 = 100 .. 103

struct st_geometry {
	int Nfft;         // FFT size of the WB numerology
	int Ngi;          // cyclic prefix, samples
	int Nc;           // active carriers
	int start_shift;  // first positive-frequency bin (cl_ofdm zero_padder)
	int n_symb;       // sounding symbols (even, comb-2 alternating)
	double fs;        // baseband sample rate the waveform is defined at (Hz)
};

// The WB numerology of physical_config.cc / telecom_system init (Nc 50).
st_geometry wb_geometry();

int  symbol_samples(const st_geometry& g);            // Nfft + Ngi
int  probe_samples(const st_geometry& g);             // n_symb * (Nfft + Ngi)
int  bin_of_carrier(const st_geometry& g, int k);     // FFT bin of carrier k (zero_padder map)
bool cell_active(const st_geometry& g, int s, int k); // carrier k sounded in symbol s
// Known cells of symbol s in carrier order (Nc entries, zero on silent carriers).
// Sum of |cell|^2 over the symbol == Nc (a data symbol of unit-energy cells).
void symbol_cells(const st_geometry& g, int s, std::complex<double>* cells);
// Unscaled baseband waveform of the whole probe at g.fs: per symbol, IFFT (1/Nfft
// normalisation) of the zero-padded cells followed by the cyclic prefix. Returns the
// number of samples written (probe_samples) or 0 when `max` is too small.
int  build_baseband(const st_geometry& g, std::complex<double>* out, int max);

// ---- measurement -----------------------------------------------------------------
struct st_measurement {
	bool   valid;
	int    Nc;
	double sinr_lin[kMaxNc];   // per-carrier SINR of one unit-energy data cell (linear)
	double var_sinr[kMaxNc];   // estimation variance of sinr_lin (delta method)
	double noise[kMaxNc];      // per-carrier noise power used (FFT units)
	double mean_snr_db;        // 10log10(mean_k sinr) -- the flat (band) SNR
	double sigma_mean_db;      // 1-sigma of mean_snr_db
	double noise_rel_var;      // relative variance of the pooled noise estimate
	int    noise_cells;        // silent cells behind the noise estimate
	bool   noise_colored;      // chi-square homogeneity rejected a white floor
	double timing_metric;      // normalised detection metric (0..1)
	int    timing_offset;      // baseband sample where the probe starts
	double cfo_hz;             // total frequency offset removed (coarse + fine)
	double time_var_frac;      // excess cell-to-cell variation / signal (time selectivity)
	double fd_rms_hz;          // rms Doppler implied by time_var_frac (Gaussian spectrum)
	double freq_sel_db;        // 10log10(mean gamma) - 10log10(harmonic-like min statistic)
	double delay_spread_ms;    // rms delay spread of the estimated impulse response
	bool   smoothed;           // delay-domain smoothing accepted
	double smooth_ratio;       // removed energy / energy expected from noise alone
};

// Estimate from the demodulated grid Y[s*Nc + k] (FFT outputs at the carriers, any
// consistent scale). Applies the residual common frequency slope removal itself.
// noise_shape (optional, Nc entries, mean 1): the known relative noise power per
// carrier of the receive chain (|H_rx(f_k)|^2 of the digital receive filter). Noise is
// pooled after dividing by it, so a shaped-but-otherwise-white floor keeps the full
// pooled accuracy; NULL = white.
bool estimate_from_grid(const st_geometry& g, const std::complex<double>* Y,
	st_measurement* m, const double* noise_shape = nullptr);

// Demodulate the probe grid at a known start n0 (probe's first sample) after removing
// cfo_hz: FFT window centred in the cyclic prefix (fft_backoff samples early), the
// window's linear phase removed. Y receives n_symb * Nc cells.
int  fft_backoff(const st_geometry& g);
void demod_grid(const st_geometry& g, const std::complex<double>* bb, int n0, double cfo_hz,
	std::complex<double>* Y);

// Probe front end, independent of the modem's current geometry (the responder may still
// be in the narrowband discovery geometry when the probe arrives): mix the real passband
// (modem convention: passband = Re{x e^{-i w t}}) down by fc, low-pass with the library's
// own windowed-sinc filter (pass edge = outermost probe carrier + one spacing, stop edge =
// first frequency the decimation folds onto a probe carrier), decimate to g.fs. Returns the number of
// baseband samples written. frontend_noise_shape gives |H(f_k)|^2 of that filter at the
// probe carriers (the noise shape the estimator whitens by).
int  frontend_taps(const st_geometry& g, double fs_pass, double fc, double* taps, int max,
	bool sharp = false);
int  passband_to_probe_baseband(const st_geometry& g, const double* pb, int n, double fs_pass,
	double fc, std::complex<double>* out, int max, bool sharp = false);
void frontend_noise_shape(const st_geometry& g, double fs_pass, double fc, double* shape);

// bb_sync (optional, same length/timing as bb): the same reception through the sharp
// (image-rejecting) front end; the timing search and the coarse frequency estimate run on
// it, the FFT demodulation on bb (short filter, least added delay spread).
// Full receive path on a baseband buffer at g.fs: timing search over probe starts in
// [search_start, search_start + search_len), coarse frequency estimate from the in-
// symbol repetition, demodulation, then estimate_from_grid. `bb` must hold at least
// search_start + search_len + probe_samples(g) samples.
bool estimate_from_baseband(const st_geometry& g, const std::complex<double>* bb, int n,
	int search_start, int search_len, st_measurement* m, const double* noise_shape = nullptr,
	const std::complex<double>* bb_sync = nullptr);

// Complete receive path on the real passband (the call the handshake makes): pass 1
// mixes at fc - cfo_hint_hz through the image-rejecting front end for timing + coarse
// frequency; pass 2 re-mixes with that offset removed through the short front end and
// demodulates. cfo_hint_hz is the expected BASEBAND offset (units of m->cfo_hz), e.g.
// the connect-stage estimate; 0 if unknown (capture range +-fs/Nfft = +-46.9 Hz).
// search_start/search_len are in baseband samples (g.fs) of this buffer.
bool estimate_from_passband(const st_geometry& g, const double* pb, int n, double fs_pass,
	double fc, double cfo_hint_hz, int search_start, int search_len, st_measurement* m);

// ---- EESM and election ---------------------------------------------------------
// Numerically stable EESM (log-sum-exp), result in dB; -99 when n <= 0.
double eesm_db(const double* sinr_lin, int n, double beta);
// Delta-method 1-sigma (dB) of eesm_db from var_sinr and the pooled-noise variance.
double eesm_sigma_db(const st_measurement& m, double beta);

struct st_cfg_row {
	int    cfg;
	double beta;         // EESM beta (linear SINR units)
	double knee01_db;    // AWGN Es/N0 at BLER 0.1 (the election threshold)
	double knee05_db;    // AWGN Es/N0 at BLER 0.5 (reported)
	double awgn_offset_db; // estimator effective SNR minus commanded Es/N0 on AWGN
	bool   measured;     // knee measured on the grid (false = lower-edge extrapolation)
};
const st_cfg_row* cfg_table();            // kNumOfdmCfg rows, index == cfg

// Guard for the high-order rungs (32/64-QAM floor on selective or time-varying
// channels, calibration VERDICT eesm_cfg11_17 sections 4-6).
struct st_guard {
	int    safe_top_cfg;        // top rung allowed when the guard trips (CONFIG_15)
	double max_time_var_frac;   // time selectivity above which cfg16+ is not elected
	double max_freq_sel_db;     // frequency selectivity above which cfg16+ is not elected
};

struct st_policy {
	double z;             // one-sided confidence multiplier on the EESM sigma
	int    top_cfg;       // highest OFDM rung the link may run (WB_CONFIG_MAX = 16)
	bool   wb_possible;   // both ends WB capable and not NB-only
	st_guard guard;
};
st_policy default_policy();

struct st_decision {
	bool   valid;
	bool   open_wb;
	int    rung;          // CONFIG_x, ROBUST_0 (below the cfg0 knee), or kRungNone
	int    fallback;      // CONFIG_0 (WB) / ROBUST_0 (NB)
	int    cap_cfg;       // selectivity ceiling for the gearshift climb
	double eff_db[kNumOfdmCfg];    // EESM effective SNR per config
	double sigma_db[kNumOfdmCfg];  // its 1-sigma
	double thr_db[kNumOfdmCfg];    // election threshold per config
	double eff_at_rung_db;
	double margin_db;              // z * sigma at the elected rung
	double mean_snr_db;
	bool   freq_selective;
	bool   time_selective;
};
st_decision decide(const st_measurement& m, const st_policy& p);

// ---- reply codec (24 bits) --------------------------------------------------------
// [23:19] rung code (0..17 CONFIG_x, 18..21 ROBUST_0..3, 31 none)
// [18:12] effective SNR at the rung, 0.5 dB steps, offset -20 dB (0..127)
// [11:7]  cap rung code
// [6:4]   margin, 0.25 dB steps (0..7 = 0..1.75 dB, saturating)
// [3]     valid  [2] frequency-selective  [1] time-selective  [0] spare (0)
struct st_reply {
	bool   valid;
	int    rung;
	int    cap_cfg;
	double eff_db;
	double margin_db;
	bool   freq_selective;
	bool   time_selective;
};
uint32_t encode_reply(const st_decision& d);
bool     decode_reply(uint32_t w24, st_reply* r);
void     reply_to_bytes(uint32_t w24, unsigned char* b3);
uint32_t reply_from_bytes(const unsigned char* b3);

// Default-on switch for callers that wire the probe (=0 disables):
// MERCURY_EESM_PROBE.
bool enabled();

// Self-contained tests of the library (no modem state). Returns the failure count.
int run_unit_tests();

} // namespace eesm_probe

#endif
