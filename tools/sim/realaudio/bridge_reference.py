"""Noise-reference selection for the native real-audio bridge, passed on the
command line and verified against the bridge's own attestation.

The native bridge runs under ``sudo -n nice``.  sudo resets the environment,
so a MERCURY_SIM_PSIG_* setting in the harness environment never reached the
bridge: every such cell ran the bridge's compiled default.  The harness
therefore resolves the requested mode here, passes it as explicit options, and
refuses a cell whose bridge attests a different mode (fail closed).

Modes (MERCURY_SIM_PSIG_MODE, read by the harness, never inside the bridge):
  unset | steady | bench   -> bench (default).  Mirrors the IONOS channel
        simulator: one fixed noise level per commanded SNR3k, present from the
        first sample and identical on the forward and reverse channels, set
        from the Mercury OFDM WB data power (OFDM_REF below, or
        MERCURY_SIM_OFDM_REF).  "--snr3k N" is the SNR3k Mercury's OFDM WB data
        realizes; every other waveform realizes N + 10*log10(P / OFDM_REF),
        exactly as on the bench, which adds a fixed noise level per dial and
        never measures its input.
  per-class | geometry     -> every transmission referenced to its own
        waveform class (each waveform realizes N).  Optional; not the bench.
  legacy | legacy-median   -> the old running median of all active chunks.
  fix                      -> fixed reference MERCURY_SIM_PSIG_FIX.
  peak                     -> highest chunk power.
MERCURY_SIM_FLOOR_REF: silence floor reference for per-class.
"""
import json
import math

# Mercury OFDM WB data power on the bridge (mean square, S32 full scale = 1).
# Derivation: see OFDM_REF_DERIVATION.  Re-measured per cell: the bridge
# attests the airtime-weighted power of every forward OFDM WB transmission.
OFDM_REF = 0.0204
OFDM_REF_DERIVATION = ("median forward OFDM WB data power of pinned cfgs 0/7/8/13/14/15/16 at snr3k 30 "
                       "(0.02030/0.02038/0.02038/0.02048/0.02044/0.02053/0.01864), bridge calibration 2026-09-22")
# A cell whose measured OFDM WB level is further than this from OFDM_REF is
# flagged: its OFDM data did not realize the label.  0.3 dB = the acceptance
# tolerance of the label (realized == commanded within +/-0.3 dB).
OFDM_LEVEL_TOL_DB = 0.3

MODES = ("bench", "geometry", "legacy-median", "fix", "peak")
_ALIASES = {"": "bench", "steady": "bench", "bench": "bench",
            "geometry": "geometry", "per-class": "geometry",
            "legacy": "legacy-median", "legacy-median": "legacy-median",
            "fix": "fix", "peak": "peak"}


class BridgeReferenceError(ValueError):
    """The requested noise reference is malformed or was not applied."""


def _positive(environ, name):
    raw = environ.get(name)
    try:
        value = float(raw)
    except (TypeError, ValueError):
        raise BridgeReferenceError("%s must be numeric (got %r)" % (name, raw))
    if not value > 0:
        raise BridgeReferenceError("%s must be > 0" % name)
    return value


def resolve(environ):
    """Return (argv, requested) for the bridge from ``environ``."""
    raw = (environ.get("MERCURY_SIM_PSIG_MODE") or "").strip().lower()
    if raw not in _ALIASES:
        raise BridgeReferenceError("unknown MERCURY_SIM_PSIG_MODE %r" % raw)
    mode = _ALIASES[raw]
    argv = ["--reference-mode", mode]
    requested = {"psig_mode": mode, "requested_as": raw or "unset"}
    if mode == "bench":
        ref = _positive(environ, "MERCURY_SIM_OFDM_REF") if environ.get("MERCURY_SIM_OFDM_REF") else OFDM_REF
        argv += ["--ofdm-ref", repr(ref)]
        requested["ofdm_reference_power"] = ref
        requested["ofdm_reference_derivation"] = (OFDM_REF_DERIVATION if ref == OFDM_REF
                                                  else "MERCURY_SIM_OFDM_REF override")
    if mode == "fix":
        value = _positive(environ, "MERCURY_SIM_PSIG_FIX")
        argv += ["--psig-fix", repr(value)]
        requested["psig_fix"] = value
    floor = environ.get("MERCURY_SIM_FLOOR_REF")
    if floor:
        value = _positive(environ, "MERCURY_SIM_FLOOR_REF")
        argv += ["--floor-ref", repr(value)]
        requested["floor_reference_power"] = value
    return argv, requested


def _kind_summary(kinds):
    """Compact per-direction, per-waveform-kind attestation."""
    out = {}
    for direction in ("fwd", "rev"):
        rows = (kinds or {}).get(direction) or {}
        out[direction] = {k: {"tx": v.get("transmissions"), "airtime_s": v.get("airtime_s"),
                              "signal_power": v.get("signal_power"),
                              "level_vs_ofdm_ref_db": v.get("level_vs_ofdm_ref_db"),
                              "snr3k_airtime_db": v.get("snr3k_airtime_db")}
                          for k, v in rows.items() if v.get("transmissions")}
    return out


def verify(stats_path, requested, passthrough=False):
    """Compare the bridge statsfile to ``requested``.

    Returns {"ok": bool, "reasons": [...], "warnings": [...], "attested": {...}}.
    A statsfile that is missing, unreadable or lacks the mode is a failure: a
    cell that cannot prove its reference is not scorable.  In bench mode the
    measured OFDM WB level is attested; a level more than OFDM_LEVEL_TOL_DB
    away from the reference is a warning (the label then describes a level
    Mercury did not transmit), not a failure: the channel itself was right."""
    out = {"ok": False, "reasons": [], "warnings": [], "attested": {}, "requested": requested}
    if passthrough:
        out["ok"] = True
        out["attested"] = {"psig_mode": "passthrough"}
        return out
    try:
        with open(stats_path) as fh:
            st = json.load(fh)
    except (OSError, ValueError) as exc:
        out["reasons"].append("bridge_stats_unreadable:%s" % type(exc).__name__)
        return out
    ax = st.get("axis_attestation") if isinstance(st, dict) else None
    ax = ax if isinstance(ax, dict) else {}
    bench = st.get("bench") if isinstance(st, dict) else None
    bench = bench if isinstance(bench, dict) else {}
    mode = st.get("psig_mode") if isinstance(st, dict) else None
    att = {"psig_mode": mode, "psig_mode_source": st.get("psig_mode_source"),
           "reference_mode": ax.get("reference_mode"),
           "reference_power": ax.get("reference_power"),
           "psig_fix": ax.get("psig_fix"),
           "floor_reference_power": ax.get("floor_reference_power"),
           "ofdm_reference_power": bench.get("ofdm_reference_power"),
           "ofdm_reference_source": bench.get("ofdm_reference_source"),
           "fwd_noise_variance": bench.get("fwd_noise_variance"),
           "rev_noise_variance": bench.get("rev_noise_variance")}
    out["attested"] = att
    if mode is None:
        out["reasons"].append("bridge_did_not_attest_reference_mode")
    elif mode != requested["psig_mode"]:
        out["reasons"].append("bridge_reference_mode_mismatch:requested=%s:attested=%s"
                              % (requested["psig_mode"], mode))
    if att["psig_mode_source"] not in (None, "cli"):
        out["reasons"].append("bridge_reference_mode_not_from_cli:%s" % att["psig_mode_source"])
    if requested["psig_mode"] == "fix":
        if att["reference_power"] is None or abs(float(att["reference_power"]) - requested["psig_fix"]) > 1e-12:
            out["reasons"].append("bridge_fixed_reference_mismatch:requested=%r:attested=%r"
                                  % (requested["psig_fix"], att["reference_power"]))
    if requested["psig_mode"] == "bench":
        ref = requested["ofdm_reference_power"]
        if (att["ofdm_reference_power"] is None or abs(float(att["ofdm_reference_power"]) - ref) > 1e-12
                or att["ofdm_reference_source"] != "cli"):
            out["reasons"].append("bridge_ofdm_reference_mismatch:requested=%r:attested=%r:source=%s"
                                  % (ref, att["ofdm_reference_power"], att["ofdm_reference_source"]))
        if att["fwd_noise_variance"] is None or att["fwd_noise_variance"] != att["rev_noise_variance"]:
            out["reasons"].append("bridge_bench_noise_not_identical_fwd_rev")
        kinds = _kind_summary(st.get("waveform_kinds"))
        out["waveform_kinds"] = kinds
        wb = kinds["fwd"].get("ofdm-wb")
        if wb and wb.get("signal_power"):
            delta = 10.0 * math.log10(float(wb["signal_power"]) / ref)
            out["ofdm_wb_level_delta_db"] = round(delta, 4)
            if abs(delta) > OFDM_LEVEL_TOL_DB:
                out["warnings"].append("ofdm_wb_level_off_reference:%+.3fdB" % delta)
    if "floor_reference_power" in requested:
        if att["floor_reference_power"] is None or abs(float(att["floor_reference_power"]) - requested["floor_reference_power"]) > 1e-12:
            out["reasons"].append("bridge_floor_reference_mismatch")
    out["ok"] = not out["reasons"]
    return out
