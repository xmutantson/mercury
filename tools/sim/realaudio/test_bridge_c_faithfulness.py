#!/usr/bin/env python3
"""Model-faithfulness gate for realaudio_bridge_s32_c.

The random realizations are intentionally allowed to differ for fading.  The
gate compares realized SNR3k, measured tap Doppler PSD, measured two-path delay,
passthrough bytes, and the production S32/float64 conversion bytes.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess
import tempfile
import types

import numpy as np

HERE = Path(__file__).resolve().parent

# Frozen Python oracle used by the staged PASS gate. Keeping the small model
# subset here makes this test independent of snd-aloop and of unrelated relay
# feature evolution while still comparing two implementations.
FS = 48000.0
PROFILES = {
    "wgn": None,
    "mpg": {"dtau": 0.5e-3, "fd": 0.1},
    "mpm": {"dtau": 1.0e-3, "fd": 0.5},
    "mpp": {"dtau": 2.0e-3, "fd": 1.0},
}


def ionos_wgn_to_snr3k(label_db):
    requested = float(label_db) + 4.8
    endpoint = 49.7
    return -10.0 * math.log10(10.0 ** (-requested / 10.0) +
                              10.0 ** (-endpoint / 10.0))


class Xoshiro:
    def __init__(self, seed):
        self.s = (seed * 0x9E3779B97F4A7C15) & 0xFFFFFFFFFFFFFFFF
        self.spare = None

    def _u64(self):
        self.s = (self.s + 0x9E3779B97F4A7C15) & 0xFFFFFFFFFFFFFFFF
        z = self.s
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & 0xFFFFFFFFFFFFFFFF
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & 0xFFFFFFFFFFFFFFFF
        return (z ^ (z >> 31)) & 0xFFFFFFFFFFFFFFFF

    def gauss(self):
        if self.spare is not None:
            value, self.spare = self.spare, None
            return value
        u1 = max((self._u64() >> 11) * (1.0 / 9007199254740992.0), 1e-15)
        u2 = (self._u64() >> 11) * (1.0 / 9007199254740992.0)
        magnitude = math.sqrt(-2.0 * math.log(u1))
        self.spare = magnitude * math.sin(2.0 * math.pi * u2)
        return magnitude * math.cos(2.0 * math.pi * u2)

    def seed_np(self):
        return np.random.default_rng(self._u64())


def _hilbert_fir(numtaps):
    middle = (numtaps - 1) // 2
    indices = np.arange(numtaps) - middle
    taps = np.zeros(numtaps)
    odd = indices % 2 != 0
    taps[odd] = 2.0 / (math.pi * indices[odd])
    return taps * np.hanning(numtaps), middle


class AnalyticFilter:
    def __init__(self, numtaps=129):
        self.h, self.delay = _hilbert_fir(numtaps)
        self.ntaps = len(self.h)
        self.hist = np.zeros(self.ntaps - 1)

    def process(self, x):
        buf = np.concatenate((self.hist, x))
        quadrature = np.convolve(buf, self.h, mode="full")
        start = self.ntaps - 1
        q_chunk = quadrature[start:start + len(x)]
        end = len(buf) - self.delay
        begin = end - len(x)
        if begin < 0:
            i_chunk = np.concatenate((np.zeros(-begin), buf[:end]))[:len(x)]
        else:
            i_chunk = buf[begin:end]
        self.hist = buf[-(self.ntaps - 1):]
        return i_chunk + 1j * q_chunk


GAUS_FIR_COEFFS = np.array([
    1.1755592671332046e-11, 2.0188004956137427e-10, 1.7236333623946176e-09,
    9.815423109243151e-09, 4.219820040519088e-08, 1.4693429486234634e-07,
    4.338503956649552e-07, 1.122118806000393e-06, 2.604091536729611e-06,
    5.522713327023963e-06, 1.0857390465642334e-05, 2.0011412649247494e-05,
    3.4893068706162795e-05, 5.7982477259392286e-05, 9.237678934582388e-05,
    0.00014180767284007714, 0.00021062669000635658, 0.0003037561533907518,
    0.0004266051229102419, 0.000584952237938201, 0.0007847989367901134,
    0.001032198204838678, 0.001333065243166745, 0.001692977321877409,
    0.002116970561258896, 0.002609341477645015, 0.003173460865247882,
    0.00381160700116864, 0.004524824309342139, 0.005312812558055807,
    0.006173850455688009, 0.007104756211217515, 0.008100886298035966,
    0.00915617235512072, 0.010263194925878348, 0.01141329161173599,
    0.012596696236577472, 0.013802704802828978, 0.015019863385625786,
    0.016236172665450826, 0.01743930354214716, 0.01861681819809535,
    0.019756391073966928, 0.020846024470695095, 0.021874253876569306,
    0.02283033861665188, 0.023704434009553566, 0.02448774186995001,
    0.02517263689027796, 0.025752767148979578, 0.026223127704178725,
    0.026580106921515002, 0.02682150583616039, 0.026946531447567135,
    0.026955765379786157, 0.02685110980161695, 0.026635712883536795,
    0.026313876369100774, 0.025890948056565766, 0.025373202123371387,
    0.024767710285281262, 0.024082206768594926, 0.02332494999439922,
    0.022504583735910823, 0.02163000032189053, 0.020710208229666707,
    0.019754206149457228, 0.018770865316331563, 0.01776882160593159,
    0.016756378583127243, 0.01574142238665159, 0.01473134903422507,
    0.013733004447687326, 0.012752637231260112, 0.011795863992392318,
    0.010867646776884902, 0.009972282000446704, 0.009113400098909145,
    0.008293974989634013, 0.007516342337046732, 0.006782225544921226,
    0.006092768355660706, 0.005448572920512081, 0.004849742212182516,
    0.004295925680163602, 0.003786367096475506, 0.003319953602663025,
    0.002895265044807468, 0.002510622769190125, 0.002164137144270543,
    0.0018537531721837, 0.001577293652557459, 0.001332499460868328,
    0.001117066600796574, 0.0009286797833839573, 0.0007650423737761393,
    0.0006239026277671727, 0.0005030762143350815, 0.0004004650862100223,
    0.0003140728178353742, 0.00024201657867910556, 0.00018253594974316996,
    0.00013399882249807025, 9.49046426887381e-05, 6.388527699708468e-05,
    3.970378899074398e-05, 2.1251412801076355e-05, 7.543009276742865e-06,
    -2.2887192936932196e-06, -8.999992637884094e-06,
    -1.3243447148502327e-05, -1.557613368708468e-05,
    -1.6467811426850445e-05, -1.63093528492861e-05,
    -1.5421097939455242e-05, -1.4061018704922785e-05,
    -1.243257788819571e-05, -1.0692187735418308e-05,
    -8.956195575428226e-06, -7.3073424710351094e-06,
    -5.8006591112396305e-06, -4.468779263172823e-06,
    -3.326665397016021e-06, -2.3757534892377295e-06,
    -1.6075344987453848e-06, -1.0065986371058506e-06,
    -5.531753923550552e-07, -2.252074190014706e-07,
], dtype=np.float64)


class DopplerTap:
    def __init__(self, fd_hz, rng):
        self.fd = fd_hz
        self.rng = rng
        self.update = max(1, int(round(FS / (fd_hz * 64.0))))
        self.inno_std = math.sqrt(0.5 / float(np.sum(GAUS_FIR_COEFFS ** 2)))
        self.fir_i = np.zeros(len(GAUS_FIR_COEFFS))
        self.fir_q = np.zeros(len(GAUS_FIR_COEFFS))
        for _ in GAUS_FIR_COEFFS:
            self._fir_update()
        self.g_hold = self._fir_output()
        self.pos = 0

    def _fir_update(self):
        self.fir_i = np.roll(self.fir_i, 1)
        self.fir_q = np.roll(self.fir_q, 1)
        self.fir_i[0] = self.rng.standard_normal() * self.inno_std
        self.fir_q[0] = self.rng.standard_normal() * self.inno_std

    def _fir_output(self):
        return (float(np.dot(GAUS_FIR_COEFFS, self.fir_i)) +
                1j * float(np.dot(GAUS_FIR_COEFFS, self.fir_q)))

    def advance(self, n):
        output = np.empty(n, dtype=np.complex128)
        offset = 0
        while offset < n:
            take = min(self.update - self.pos, n - offset)
            output[offset:offset + take] = self.g_hold
            offset += take
            self.pos += take
            if self.pos == self.update:
                self.pos = 0
                self._fir_update()
                self.g_hold = self._fir_output()
        return output


class Channel:
    def __init__(self, channel_args, rng_seed):
        self.snr_db = channel_args.snr
        self.snr_lin = 10.0 ** (self.snr_db / 10.0)
        self.sig_ref = channel_args.sig_ref
        self.rng = Xoshiro(rng_seed)
        np_rng = self.rng.seed_np()
        self.p_sig = max(self.sig_ref, 1e-6) ** 2
        self.noise_std = self._noise_std(self.p_sig)
        self.active_ms = []
        self.profile = channel_args.profile
        profile = PROFILES[self.profile]
        self.fading = profile is not None
        if self.fading:
            self.delay = int(round(profile["dtau"] * FS))
            self.analytic = AnalyticFilter()
            self.tap0 = DopplerTap(profile["fd"], np_rng)
            self.tap1 = DopplerTap(profile["fd"], np_rng)
            self.delay_buf = np.zeros(self.delay, dtype=np.complex128)

    def _noise_std(self, power):
        return math.sqrt(power * 24000.0 / (self.snr_lin * 3000.0))

    def process(self, samples):
        x = np.asarray(samples, dtype=np.float64)
        ms = float(np.mean(x * x)) if x.size else 0.0
        if ms > 1e-7:
            self.active_ms.append(ms)
            if len(self.active_ms) % 8 == 0:
                self.p_sig = float(np.median(self.active_ms))
                self.noise_std = self._noise_std(self.p_sig)
        if self.fading:
            z = self.analytic.process(x)
            delayed = np.empty(x.size, dtype=np.complex128)
            if x.size >= self.delay:
                delayed[:self.delay] = self.delay_buf
                delayed[self.delay:] = z[:-self.delay]
                self.delay_buf = z[-self.delay:].copy()
            else:
                delayed[:] = self.delay_buf[:x.size]
                self.delay_buf = np.concatenate((self.delay_buf[x.size:], z))
            y = (self.tap0.advance(x.size) * z +
                 self.tap1.advance(x.size) * delayed) / math.sqrt(2.0)
            output = np.real(y)
        else:
            output = x.copy()
        noise = np.fromiter((self.rng.gauss() for _ in range(x.size)),
                            dtype=np.float64, count=x.size)
        return (output + self.noise_std * noise).tolist()

BIN = HERE / "realaudio_bridge_s32_c"
INT_MAX = 2147483647.0
PERIOD = 1024
SNR_TOL_DB = 0.1
PSD_RMSE_TOL_DB = 0.85
DOPPLER_SPREAD_TOL_PCT = 4.0


def args(profile="wgn", snr=30.0, sig_ref=0.15):
    return types.SimpleNamespace(
        snr=float(snr), loss=0.0, burst=False, profile=profile,
        cfo_hz=0.0, phase_noise_deg=0.0, sig_ref=float(sig_ref),
        fade_depth_db=0.0, bandpass_lo_hz=0.0,
        bandpass_hi_hz=0.0, bandpass_taps=511,
    )


def run_c_double(x, *opts):
    with tempfile.TemporaryDirectory(prefix="bridge_c_test_") as td:
        pi, po = Path(td) / "in.f64", Path(td) / "out.f64"
        np.asarray(x, dtype="<f8").tofile(pi)
        subprocess.run([str(BIN), "--double-in", str(pi), "--double-out", str(po), *opts],
                       check=True, stdout=subprocess.DEVNULL)
        return np.fromfile(po, dtype="<f8")


def run_c_vector(stereo, *opts):
    with tempfile.TemporaryDirectory(prefix="bridge_c_test_") as td:
        pi, po = Path(td) / "in.s32", Path(td) / "out.s32"
        np.asarray(stereo, dtype="<i4").tofile(pi)
        subprocess.run([str(BIN), "--vector-in", str(pi), "--vector-out", str(po), *opts],
                       check=True, stdout=subprocess.DEVNULL)
        return np.fromfile(po, dtype="<i4")


def py_process(x, profile, snr, seed=11):
    ch = Channel(args(profile, snr), (seed * 2654435761) & 0xFFFFFFFF)
    chunks = []
    for i in range(0, len(x), PERIOD):
        chunks.append(np.asarray(ch.process(x[i:i + PERIOD].tolist()), dtype=np.float64))
    return np.concatenate(chunks)


def snr3k(sig, out):
    noise = out - sig
    # Invert Channel._noise_std_from_psig(): var=P_sig*24000/(SNR*3000).
    return 10.0 * math.log10(float(np.mean(sig * sig)) * 8.0 / float(np.mean(noise * noise)))


def snr_gate(report):
    n = PERIOD * 256
    t = np.arange(n, dtype=np.float64) / FS
    x = 0.15 * math.sqrt(2.0) * np.sin(2.0 * math.pi * 1500.0 * t)
    rows = []
    for label in (10, 20, 30, 40):
        mapped = ionos_wgn_to_snr3k(label)
        yp = py_process(x, "wgn", mapped)
        yc = run_c_double(x, "--cell", f"WGN:{label}", "--profile", "wgn",
                          "--sig-ref", "0.15", "--seed", "11")
        sp, sc = snr3k(x, yp), snr3k(x, yc)
        delta = abs(sp - sc)
        rows.append({"dial": label, "mapped_snr3k_db": mapped,
                     "python_realized_db": sp, "c_realized_db": sc,
                     "c_minus_python_db": sc - sp, "abs_delta_db": delta,
                     "pass": delta <= SNR_TOL_DB})
    report["snr"] = rows
    return all(r["pass"] for r in rows)


def format_gates(report):
    rg = np.random.default_rng(0xC0DEC)
    ch0 = np.concatenate((np.array([np.iinfo(np.int32).min, -2147483647, -1, 0, 1,
                                    2147483646, 2147483647], dtype=np.int32),
                          rg.integers(-2**31, 2**31, 100003, dtype=np.int32)))
    ch1 = rg.integers(-2**31, 2**31, ch0.size, dtype=np.int32)
    inp = np.column_stack((ch0, ch1)).astype("<i4").ravel()

    got_pass = run_c_vector(inp, "--passthrough")
    expect_pass = np.repeat(ch0, 2).astype("<i4")
    pass_equal = bool(np.array_equal(got_pass, expect_pass))

    xf = ch0.astype(np.float64) / INT_MAX
    scaled = xf * INT_MAX
    np.clip(scaled, -INT_MAX, INT_MAX, out=scaled)
    q = scaled.astype("<i4")
    expect_fmt = np.repeat(q, 2)
    got_fmt = run_c_vector(inp, "--format-only")
    fmt_equal = bool(np.array_equal(got_fmt, expect_fmt))
    report["passthrough"] = {"frames": int(ch0.size), "byte_exact": pass_equal,
                             "mismatched_words": int(np.count_nonzero(got_pass != expect_pass))}
    report["format"] = {"frames": int(ch0.size), "byte_exact": fmt_equal,
                        "mismatched_words": int(np.count_nonzero(got_fmt != expect_fmt)),
                        "int32_min_expected_after_channel_path": int(q[0])}
    return pass_equal and fmt_equal


def welch(z, nper=4096):
    z = np.asarray(z, dtype=np.complex128)
    win = np.hanning(nper)
    acc = None
    count = 0
    for start in range(0, len(z) - nper + 1, nper // 2):
        v = z[start:start+nper] - np.mean(z[start:start+nper])
        p = np.abs(np.fft.fftshift(np.fft.fft(v * win))) ** 2
        acc = p if acc is None else acc + p
        count += 1
    p = acc / count
    p /= np.sum(p)
    return p


def c_taps(profile, count, seed):
    fd = PROFILES[profile]["fd"]
    expected_stride = int(round(FS/(64.0*fd)))
    with tempfile.TemporaryDirectory(prefix="bridge_c_taps_") as td:
        po = Path(td) / "tap.f64"
        subprocess.run([str(BIN), "--profile", profile, "--seed", str(seed),
                        "--tap-count", str(count), "--tap-stride", str(expected_stride),
                        "--tap-out", str(po)], check=True)
        v = np.fromfile(po, dtype="<f8").reshape(-1, 2)
        return v[:, 0] + 1j * v[:, 1]


def py_taps(profile, count, seed):
    fd = PROFILES[profile]["fd"]
    tap = DopplerTap(fd, np.random.default_rng(seed))
    out = np.empty(count, dtype=np.complex128)
    # Equivalent to advance(update): emit the held value, then one FIR update.
    for i in range(count):
        out[i] = tap.g_hold
        tap._fir_update()
        tap.g_hold = tap._fir_output()
    return out


def theoretical_psd(nper):
    h = np.zeros(nper)
    h[:len(GAUS_FIR_COEFFS)] = GAUS_FIR_COEFFS
    p = np.abs(np.fft.fftshift(np.fft.fft(h))) ** 2
    return p / np.sum(p)


def estimate_delay(y, impulse_at=256):
    # Build the exact Python analytic impulse template.  For each candidate
    # two-path delay, least-squares fit independent complex tap coefficients.
    x = np.zeros(PERIOD)
    x[impulse_at] = 0.2
    z = AnalyticFilter(129).process(x)
    best = (float("inf"), None)
    for d in range(1, 193):
        zd = np.zeros_like(z)
        zd[d:] = z[:-d]
        A = np.column_stack((z.real, -z.imag, zd.real, -zd.imag))
        coef, *_ = np.linalg.lstsq(A, y, rcond=None)
        residual = float(np.mean((y - A @ coef) ** 2))
        if residual < best[0]:
            best = residual, d
    return int(best[1]), best[0]


def fade_gate(report):
    count = 262144
    nper = 4096
    pth = theoretical_psd(nper)
    keep = 10 * np.log10(np.maximum(pth / np.max(pth), 1e-300)) > -35
    x = np.zeros(PERIOD)
    x[256] = 0.2
    rows = []
    for profile in ("mpg", "mpm", "mpp"):
        fd = PROFILES[profile]["fd"]
        zp, zc = py_taps(profile, count, 1234), c_taps(profile, count, 1234)
        pp, pc = welch(zp, nper), welch(zc, nper)
        # Remove arbitrary total gain and compare the measured PSD shapes.
        pp_db = 10*np.log10(np.maximum(pp/np.max(pp), 1e-300))
        pc_db = 10*np.log10(np.maximum(pc/np.max(pc), 1e-300))
        rmse = float(np.sqrt(np.mean((pp_db[keep] - pc_db[keep]) ** 2)))
        freq = np.fft.fftshift(np.fft.fftfreq(nper, d=1.0/(64.0*fd)))
        region = np.abs(freq) <= 4.0*fd
        rms_p = float(np.sqrt(np.sum(freq[region]**2*pp[region])/np.sum(pp[region])))
        rms_c = float(np.sqrt(np.sum(freq[region]**2*pc[region])/np.sum(pc[region])))
        spread_delta = abs(rms_c-rms_p)/rms_p*100.0

        yp = py_process(x, profile, 300.0, seed=19)
        yc = run_c_double(x, "--profile", profile, "--snr", "300", "--seed", "19")
        dp, rp = estimate_delay(yp)
        dc, rc = estimate_delay(yc)
        target = int(round(PROFILES[profile]["dtau"]*FS))
        ok = (rmse <= PSD_RMSE_TOL_DB and spread_delta <= DOPPLER_SPREAD_TOL_PCT
              and dp == target and dc == target)
        rows.append({"profile": profile.upper(), "fd_hz": fd,
                     "tap_update_samples": int(round(FS/(64*fd))),
                     "doppler_psd_rmse_db": rmse,
                     "python_rms_doppler_hz": rms_p, "c_rms_doppler_hz": rms_c,
                     "rms_doppler_delta_pct": spread_delta,
                     "target_delay_samples": target, "python_delay_samples": dp,
                     "c_delay_samples": dc, "python_delay_fit_mse": rp,
                     "c_delay_fit_mse": rc, "pass": ok})
    report["fade"] = rows
    return all(r["pass"] for r in rows)


def attestation_gate(report):
    with tempfile.TemporaryDirectory(prefix="bridge_c_attest_") as td:
        p = Path(td) / "stats.json"
        subprocess.run([str(BIN), "--dry-run", "--cell", "WGN:40", "--profile", "wgn",
                        "--seed", "7", "--statsfile", str(p)], check=True)
        obj = json.loads(p.read_text())
    att = obj["channel_attestation"]
    required = {"cell", "profile", "commanded_snr", "realized_snr3k",
                "realized_snr_offset_db", "seed", "passthrough", "realized_p_sig"}
    ok = required <= set(att) and att["cell"] == "WGN:40" and att["profile"] == "WGN"
    ok = ok and att["seed"] == 7 and abs(att["realized_p_sig"] - .15**2) < 1e-12
    report["attestation"] = {"pass": bool(ok), **att}
    axis = obj.get("axis_attestation", {})
    axis_required = {
        "axis_version", "input_coordinate", "reference_mode", "reference_id",
        "reference_power", "reference_n_samples", "configured_bandwidth_hz",
        "snr3k_db", "cn_config_db", "noise_variance", "seed",
        "composite_scale", "pre_scale_peak", "hard_clip_count",
        "s32_saturation_count", "binary_sha256", "recipe_sha256",
    }
    axis_ok = (axis_required <= set(axis)
               and axis.get("axis_version") == "v1-steady-snr3k"
               and axis.get("input_coordinate") == "cell"
               and axis.get("seed") == 7
               and abs(axis.get("reference_power", 0.0) - .15**2) < 1e-12)
    report["axis_attestation"] = {"pass": bool(axis_ok), **axis}
    return bool(ok and axis_ok)


def main():
    if not BIN.exists():
        subprocess.run(["make", "-C", str(HERE), "realaudio_bridge_s32_c"], check=True)
    report = {"thresholds": {"snr_abs_delta_db": SNR_TOL_DB,
                              "doppler_psd_rmse_db": PSD_RMSE_TOL_DB,
                              "doppler_spread_delta_pct": DOPPLER_SPREAD_TOL_PCT}}
    gates = {
        "snr": snr_gate(report),
        "fade": fade_gate(report),
        "format_passthrough": format_gates(report),
        "attestation": attestation_gate(report),
    }
    report["gates"] = gates
    report["faithfulness"] = "PASS" if all(gates.values()) else "FAIL"
    out = HERE / "faithfulness_results.json"
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(f"FAITHFULNESS {report['faithfulness']}")
    for row in report["snr"]:
        print(f"SNR WGN:{row['dial']:2d} Python={row['python_realized_db']:.4f} "
              f"C={row['c_realized_db']:.4f} delta={row['abs_delta_db']:.4f} dB")
    for row in report["fade"]:
        print(f"FADE {row['profile']} PSD_RMSE={row['doppler_psd_rmse_db']:.3f} dB "
              f"RMS_delta={row['rms_doppler_delta_pct']:.2f}% "
              f"delay py/C/target={row['python_delay_samples']}/"
              f"{row['c_delay_samples']}/{row['target_delay_samples']} samples")
    print(f"PASSTHROUGH exact={report['passthrough']['byte_exact']} "
          f"FORMAT exact={report['format']['byte_exact']} "
          f"ATTESTATION={report['attestation']['pass']} "
          f"AXIS_ATTESTATION={report['axis_attestation']['pass']}")
    print(f"report={out}")
    return 0 if report["faithfulness"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
