#!/usr/bin/env python3
"""Harness identity attestation and the bench [TxGain] calibration path.

Two small jobs shared by both real-audio harness lineages:

1. Identity.  The harness hashes the bridge program it launches and its own
   source file, passes both (plus a lineage name) to the native bridge, and
   checks that the bridge echoed the same values into its statsfile
   ``axis_attestation`` block.  A result row then names exactly which bridge
   binary and which harness file produced it.

2. Bench transmit calibration (case B).  The IONOS bench Pis carry a
   ``[TxGain]`` section in ``~/.config/mercury/mercury.ini`` that lowers the
   MFSK / ACK / BREAK transmit gains so every waveform lands at the bench's
   calibrated level (plan section 7.13.21).  The sim twin reproduces that by
   giving each modem a private HOME whose settings file carries the same
   ``[TxGain]`` keys, then checks the modem log for one
   ``[TX-GAIN-OVERRIDE]`` line per key on each peer.

The key set and the INI grammar mirror the modem's own settings loader:
  keys    source/gui/ini_parser.cc  k_tx_gain_sig_names x k_tx_gain_mode_names
          ("MFSK_1S", "MFSK_2S", "OFDM", "ACK", "BREAK") x ("WB", "NB"),
          read as ini.getDouble("TxGain", "<SIG>_<MODE>")
  path    getDefaultConfigPath(): $HOME/.config/mercury/mercury.ini
  grammar IniParser::load: trimmed lines, ';' or '#' comments,
          "[Section]" headers, "key = value" pairs, last value wins
  print   cl_telecom_system::set_tx_gain:
          "[TX-GAIN-OVERRIDE] <sig padded>  <WB|NB>  %.4f -> %.4f ..."
"""
import hashlib
import math
import os
import re

TX_GAIN_SECTION = "TxGain"
TX_GAIN_SIGNALS = ("MFSK_1S", "MFSK_2S", "OFDM", "ACK", "BREAK")
TX_GAIN_MODES = ("WB", "NB")
TX_GAIN_KEYS = tuple("%s_%s" % (sig, mode)
                     for sig in TX_GAIN_SIGNALS for mode in TX_GAIN_MODES)
TX_GAIN_ENV = "MERCURY_SIM_TXGAIN_INI"
MODEM_INI_RELPATH = os.path.join(".config", "mercury", "mercury.ini")

# set_tx_gain prints the new gain with %.4f, so a printed value can differ from
# the requested one by at most half of the last printed digit.
OVERRIDE_PRINT_TOLERANCE = 0.5e-4

SHA256_HEX_RE = re.compile(r"^[0-9a-f]{64}$")
OVERRIDE_RE = re.compile(
    r"(?:\[(?P<label>RSP|CMD)\]\s+)?\[TX-GAIN-OVERRIDE\]\s+"
    r"(?P<sig>[A-Z0-9_]+)\s+(?P<mode>WB|NB)\s+"
    r"(?P<prev>-?[0-9]+(?:\.[0-9]+)?)\s+->\s+(?P<new>-?[0-9]+(?:\.[0-9]+)?)")
INI_DB_RE = re.compile(
    r"(?:\[(?P<label>RSP|CMD)\]\s+)?\[TX-GAIN\] INI:\s+(?P<db>-?[0-9]+(?:\.[0-9]+)?) dB")


class TxGainError(ValueError):
    """The [TxGain] INI is unusable or the modem did not apply it."""


def sha256_file(path, block_size=1 << 20):
    """Return the hex SHA-256 of ``path``."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(block_size), b""):
            digest.update(chunk)
    return digest.hexdigest()


# ---------------------------------------------------------------- identity

def identity(bridge_path, harness_path, lineage):
    """Hash the bridge program and the harness file for the attestation."""
    if not lineage or len(lineage) >= 32:
        raise ValueError("harness lineage must be 1..31 characters")
    return {
        "bridge_sha256": sha256_file(bridge_path),
        "harness_lineage": lineage,
        "harness_sha256": sha256_file(os.path.abspath(harness_path)),
    }


def bridge_identity_argv(ident):
    """Bridge options that carry the identity into the statsfile."""
    return ["--bridge-sha256", ident["bridge_sha256"],
            "--harness-lineage", ident["harness_lineage"],
            "--harness-sha256", ident["harness_sha256"]]


def bridge_accepts_identity(bridge_path):
    """Only the native bridge takes the identity options; a .py bridge does not."""
    return not bridge_path.endswith(".py")


def verify_identity(attestation, ident):
    """Return the list of reasons the bridge echo differs from ``ident``."""
    reasons = []
    for field in ("bridge_sha256", "harness_lineage", "harness_sha256"):
        got = attestation.get(field)
        if got != ident[field]:
            reasons.append("%s_mismatch" % field)
    for field in ("bridge_sha256", "harness_sha256"):
        if not SHA256_HEX_RE.match(str(attestation.get(field, ""))):
            reasons.append("%s_not_hex" % field)
    return reasons


# ---------------------------------------------------------------- [TxGain]

def resolve_tx_gain_ini(cli_value, environ):
    """CLI --tx-gain-ini wins; else MERCURY_SIM_TXGAIN_INI; empty means unset."""
    if cli_value:
        return cli_value, "cli"
    env_value = environ.get(TX_GAIN_ENV, "")
    if env_value:
        return env_value, "env"
    return None, None


def parse_ini_text(text):
    """Parse INI text with the modem's IniParser rules; returns {section: {k: v}}."""
    data = {}
    section = "General"
    for raw in text.splitlines():
        line = raw.strip(" \t\r\n")
        if not line or line[0] in ";#":
            continue
        if line[0] == "[" and line[-1] == "]":
            section = line[1:-1]
            continue
        if "=" in line:
            key, value = line.split("=", 1)
            data.setdefault(section, {})[key.strip(" \t\r\n")] = value.strip(" \t\r\n")
    return data


def read_tx_gain_ini(path):
    """Return {key: gain} from the [TxGain] section of ``path``.

    Fails closed on anything the modem would silently ignore or misread: a
    missing or empty section, a key the loader does not read, or a value that
    is not a finite positive number.
    """
    with open(path, "r", encoding="utf-8") as handle:
        sections = parse_ini_text(handle.read())
    raw = sections.get(TX_GAIN_SECTION)
    if not raw:
        raise TxGainError("%s: no [%s] entries" % (path, TX_GAIN_SECTION))
    unknown = sorted(set(raw) - set(TX_GAIN_KEYS))
    if unknown:
        raise TxGainError("%s: [%s] keys the modem does not read: %s"
                          % (path, TX_GAIN_SECTION, ", ".join(unknown)))
    values = {}
    for key in TX_GAIN_KEYS:
        if key not in raw:
            continue
        try:
            gain = float(raw[key])
        except ValueError:
            raise TxGainError("%s: [%s] %s=%r is not a number"
                              % (path, TX_GAIN_SECTION, key, raw[key]))
        if not math.isfinite(gain) or gain <= 0.0:
            raise TxGainError("%s: [%s] %s=%r is not a finite positive gain"
                              % (path, TX_GAIN_SECTION, key, raw[key]))
        values[key] = gain
    return values


def render_modem_settings(values):
    """Settings file body: the [TxGain] keys only, in the loader's key order."""
    lines = ["[%s]" % TX_GAIN_SECTION]
    for key in TX_GAIN_KEYS:
        if key in values:
            lines.append("%s=%s" % (key, repr(float(values[key]))))
    return "\n".join(lines) + "\n"


def write_modem_settings(home_dir, values):
    """Write $home_dir/.config/mercury/mercury.ini for one modem; return its path."""
    path = os.path.join(home_dir, MODEM_INI_RELPATH)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(render_modem_settings(values))
    return path


def parse_override_lines(log_text):
    """Every [TX-GAIN-OVERRIDE] line: dicts with label, key, prev, new."""
    rows = []
    for match in OVERRIDE_RE.finditer(log_text):
        rows.append({
            "label": match.group("label"),
            "key": "%s_%s" % (match.group("sig"), match.group("mode")),
            "prev": float(match.group("prev")),
            "new": float(match.group("new")),
        })
    return rows


def parse_ini_db_lines(log_text):
    """Every "[TX-GAIN] INI: <x> dB" line: {label: db} (last one per label)."""
    out = {}
    for match in INI_DB_RE.finditer(log_text):
        out[match.group("label")] = float(match.group("db"))
    return out


def check_tx_gain_log(log_text, values, labels=("RSP", "CMD")):
    """Attest the modem log against the requested [TxGain] values.

    ``values`` = {} is the default path: no override line may appear.  With
    values, each peer must print exactly the requested keys at the requested
    gain; ``applied_db`` is 20*log10(new/prev) per key.
    """
    rows = parse_override_lines(log_text)
    ini_db = parse_ini_db_lines(log_text)
    reasons = []
    applied = {}
    for label in labels:
        seen = {}
        for row in rows:
            if row["label"] in (label, None):
                seen[row["key"]] = row
        for key, want in sorted(values.items()):
            row = seen.get(key)
            if row is None:
                reasons.append("%s:%s_override_missing" % (label, key))
                continue
            if abs(row["new"] - want) > OVERRIDE_PRINT_TOLERANCE:
                reasons.append("%s:%s_override_%.4f_expected_%.4f"
                               % (label, key, row["new"], want))
            applied.setdefault(key, {})[label] = {
                "prev": row["prev"], "new": row["new"],
                "applied_db": (round(20.0 * math.log10(row["new"] / row["prev"]), 3)
                               if row["prev"] > 0 and row["new"] > 0 else None),
            }
        for key in sorted(set(seen) - set(values)):
            reasons.append("%s:%s_unexpected_override" % (label, key))
        # The settings file never carries [GUI] TxGainDb, so the modem's
        # overall INI transmit gain must read 0.0 dB on both paths.
        if label not in ini_db:
            reasons.append("%s:tx_gain_ini_db_line_missing" % label)
        elif ini_db[label] != 0.0:
            reasons.append("%s:tx_gain_ini_db_%.1f_not_zero" % (label, ini_db[label]))
    return {
        "mode": "ini" if values else "default",
        "requested": dict(sorted(values.items())),
        "override_lines": len(rows),
        "applied": applied,
        "ini_db": ini_db,
        "ok": not reasons,
        "reasons": reasons,
    }
