#!/usr/bin/env python3
"""
capstone_arms.py - shared arm-spec + anatomy + VARA-bar module for the faithful
real-audio CAPSTONE integration harness.

This is the ONLY new-logic module added on top of the proven clean-A/B ruler
(arq_realaudio.py + parallel_spawner.py + realaudio_bridge_s32_c +
sim_channel_relay.py). It is measurement infra ONLY - it never touches a mercury
RX/ACK/PHY source line. It provides the four capstone capabilities as reusable
helpers so the per-cell runner and the cohort spawner share ONE copy:

  (a) PER-LEVER KILL-SWITCHES  -> LEVERS registry + resolve_arm()/ARMS presets.
      An arm is a set of named levers toggled on/off; each lever maps to the
      EXACT env var(s) applied to BOTH mercury instances. ALL-OFF injects
      NOTHING, so it reproduces the stock monitor baseline byte-for-byte.

  (b) DELIVERED-PAYLOAD BYTE-INTEGRITY -> expected_slice()/tx_chunk(). The TX
      stream is the deterministic pattern bytes(range(256))*8 (byte[k]==k%256);
      the RX side compares each delivered segment against expected_slice() and
      HARD-FAILS on any mismatch/truncation. (The only catch for a
      silent false-accept; runs on every cell, every arm.)

  (c) ANATOMY LINE-ITEMS -> compute_inburst() ported VERBATIM from the proven
      tools/run_inband_ab.py anatomy profiler, plus MARKERS (the datalink
      line-item regexes: retx rounds, demote, BREAK, SET_CONFIG, RX-TIMEOUT).

  (d) VARA-BAR reference -> VARA_CLIENT_BMIN + vara_bar()/snr3k_from_cell().
      Owner-canonical client-effective B/min by SNR3k (Muething 2025 worksheet,
      bigblock_p3_hw/_vara + _research/capstone/setup.json).
"""
import os
import re
import statistics
import sys

_PARENT_SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_SIM not in sys.path:
    sys.path.insert(0, _PARENT_SIM)
from sim_channel_relay import ionos_wgn_to_snr3k  # noqa: E402


# ==========================================================================
# (a) PER-LEVER KILL-SWITCHES
# --------------------------------------------------------------------------
# Each lever maps a human name -> the exact env toggle(s) applied to BOTH
# mercury instances. `kind` documents HOW the lever is controlled; `probe` is
# the literal to grep in the binary to confirm a RUNTIME env is actually
# compiled in (probe_binary() below), so an inert/absent lever is flagged
# instead of silently doing nothing. `status` is the source-of-truth note that
# feeds the "gap" deliverable.
#
# Env var names were confirmed by grepping mercury/source (getenv sites) and by
# probing the deployed capstone binaries (mercury_trio 2cf0c48d / mercury_legacy
# e16a73b7) for each literal - see the module-header of the returned harness
# report for the file:line evidence.
# ==========================================================================
LEVERS = {
    # T1 deterministic-TDD-ACK-slot (kill the fixed ~4.6s listen guard).
    "ack_slot": {
        "env": {"MERCURY_ACK_SLOT": "1"},
        "kind": "runtime",
        "probe": "MERCURY_ACK_SLOT",
        "desc": "T1 deterministic TDD ACK slot (retire the fixed listen guard)",
        "status": "ABSENT in the monitor/2cf0c48d binaries (grep-verified: no "
                  "getenv(\"MERCURY_ACK_SLOT\"), literal absent from mercury_trio). "
                  "The env is INERT until the T1 lever lands; name is the intended "
                  "toggle so the arm-spec is ready when it does.",
    },
    # Compact coded reverse ACK (GF(16) suffix).
    "compact_confirm": {
        "env": {"ARQ_COMPACT_CONFIRM_ENABLE": "1"},
        "kind": "compile-time",
        "probe": None,          # compile-time #define, not a runtime literal
        "desc": "compact-confirm coded reverse ACK (GF(16) suffix; held-off default)",
        "status": "COMPILE-TIME symbol (ARQ_COMPACT_CONFIRM_ENABLE, held-off "
                  "default 0 in arq_commander.cc:3897/4602). NOT a getenv - the env "
                  "is a NO-OP on a stock build; enabling it needs a rebuild with "
                  "-DARQ_COMPACT_CONFIRM_ENABLE=1. The arm-spec records the request "
                  "and the runner flags it as a build-time lever.",
    },
    # HARQ chase-combining of failed data frames (T4).
    "harq_chase": {
        "env": {"MERCURY_HARQ_CHASE": "1"},
        "kind": "absent",
        "probe": "MERCURY_HARQ_CHASE",
        "desc": "HARQ chase-combining of failed data frames (T4)",
        "status": "ABSENT - no HARQ / chase-combining of DATA frames exists in the "
                  "codebase (grep-verified). "
                  "The only 'combining' in-source is base-pattern ctrl-suffix "
                  "combining (data_container.cc/arq_common.cc, a different thing). "
                  "MERCURY_HARQ_CHASE is a PLACEHOLDER name - rewire to the real env "
                  "when the T4 lever is implemented.",
    },
    # In-band CONFIG_TAG rate adapt / SUPER-ACK (receiver-drives-rate) stack.
    "inband_rate": {
        "env": {"MERCURY_INBAND_RATE": "1"},
        "kind": "runtime",
        "probe": "MERCURY_INBAND_RATE",
        "desc": "in-band CONFIG_TAG rate-adapt / SUPER-ACK receiver-drives-rate stack",
        "status": "LIVE runtime env (arq_common.cc:2553 + many arq_responder.cc "
                  "sites; literal present in mercury_trio). Also the MASTER GATE "
                  "under which the reverse-pin (Part A) is active.",
    },
    # Reverse-ACK PIN to ROBUST_1 on the CONFIG_TAG tier-cross (Part A).
    "reverse_pin": {
        "env": {},              # no clean runtime ENABLE; default-on under inband
        "kind": "default-on/compile-time-off",
        "probe": "INBAND_REVERSE_PIN",
        "desc": "reverse-ACK PIN to ROBUST_1 on the CONFIG_TAG cross (Part A)",
        "status": "DEFAULT-ON in the binary and rides inband_rate (Part A merged). "
                  "There is NO runtime env to DISABLE it - only a compile-time "
                  "FAIL-BEFORE (-DINBAND_REVERSE_PIN_FAILBEFORE, arq_commander.cc:10506) "
                  "reverts it. Modelled here as a rider on inband_rate; turning it "
                  "OFF for an A/B needs a fail-before build.",
    },
    # Time-interpolation channel-estimator seed (deep-DSP TINTERP-SEED win).
    "tinterp_seed": {
        "env": {"MERCURY_TINTERP_SEED": "1"},
        "kind": "runtime",
        "probe": "MERCURY_TINTERP_SEED",
        "desc": "time-interpolation channel-estimator seed (deep-DSP TINTERP-SEED)",
        "status": "LIVE runtime env in mercury_trio/2cf0c48d (probe-confirmed; "
                  "telecom_system.cc getenv on the revack-cc/trio lineage). May be "
                  "ABSENT in older monitor baselines - probe the manifest binary.",
    },
}

# Named preset arms. An arm is {lever_name: bool}. Absent keys == OFF.
# ALL-OFF ("all_off") is the baseline: it injects zero env and MUST reproduce the
# stock monitor binary byte-for-byte.
ARMS = {
    "all_off":        {},
    "inband_only":    {"inband_rate": True},
    "t1_ack_slot":    {"ack_slot": True},
    "inband_t1":      {"inband_rate": True, "ack_slot": True},
    "tinterp":        {"tinterp_seed": True},
    "inband_tinterp": {"inband_rate": True, "tinterp_seed": True},
    # everything the manifest binary can carry; reverse_pin rides inband_rate.
    "full_stack":     {"inband_rate": True, "ack_slot": True, "compact_confirm": True,
                       "harq_chase": True, "tinterp_seed": True, "reverse_pin": True},
}


def normalize_spec(spec):
    """Accept a dict {lever: bool}, a list of ON lever names, or a preset arm
    name (str). Returns a dict {lever: bool} with every known lever present."""
    if spec is None:
        spec = {}
    if isinstance(spec, str):
        if spec not in ARMS:
            raise ValueError(f"unknown preset arm '{spec}'; known: {sorted(ARMS)}")
        spec = ARMS[spec]
    if isinstance(spec, (list, tuple, set)):
        spec = {name: True for name in spec}
    out = {lever: False for lever in LEVERS}
    for k, v in spec.items():
        if k not in LEVERS:
            raise ValueError(f"unknown lever '{k}'; known: {sorted(LEVERS)}")
        out[k] = bool(v)
    return out


def resolve_arm(spec):
    """Resolve an arm-spec -> (env_kv_list, warnings).

    env_kv_list is a sorted list of "KEY=VAL" strings to inject into BOTH mercury
    instances (feeds arq_realaudio's existing --env). warnings lists every ON
    lever that CANNOT be toggled purely by env (compile-time / absent / rider),
    so the runner can flag it loudly instead of silently doing nothing.
    ALL-OFF -> ([], [])  (baseline; nothing injected)."""
    norm = normalize_spec(spec)
    env = {}
    warnings = []
    for lever, on in norm.items():
        if not on:
            continue
        info = LEVERS[lever]
        if info["env"]:
            env.update(info["env"])
        if info["kind"] != "runtime":
            warnings.append(f"{lever}: {info['kind']} - {info['status']}")
    kv = [f"{k}={v}" for k, v in sorted(env.items())]
    return kv, warnings


def probe_binary(binary_path):
    """Grep a mercury binary for each lever's `probe` literal. Returns
    {lever: True/False/None} (None = not probeable, e.g. compile-time). Used to
    warn when a requested RUNTIME lever's env string is absent from the binary
    (inert). Pure stdlib; reads the binary in chunks."""
    present = {lever: None for lever in LEVERS}
    needles = {lever: info["probe"].encode()
               for lever, info in LEVERS.items() if info.get("probe")}
    if not needles:
        return present
    found = set()
    try:
        with open(binary_path, "rb") as f:
            tail = b""
            while True:
                chunk = f.read(1 << 20)
                if not chunk:
                    break
                buf = tail + chunk
                for lever, needle in needles.items():
                    if lever not in found and needle in buf:
                        found.add(lever)
                tail = buf[-64:]     # carry a small overlap for split needles
    except OSError:
        return present
    for lever in needles:
        present[lever] = lever in found
    return present


# ==========================================================================
# (b) DELIVERED-PAYLOAD BYTE-INTEGRITY  (+ UNIQUENESS, capstone STEP 2c)
# --------------------------------------------------------------------------
# The canonical TX stream is a PURE FUNCTION of the absolute byte offset k, so
# TX (arq_realaudio.tx_thread_fn) and the RX verifier can never drift. For 253
# of every 256 bytes it is the original deterministic pattern byte[k]==k%256
# (the "existing byte-integrity assertion" the campaign has always used). The
# THREE bytes at each 256-block boundary (block-offsets 0,1,2) instead carry the
# little-endian 24-bit BLOCK INDEX (k//256), which makes the whole sequence
# NON-PERIODIC over a period of 256*2^24 = 4.29e9 bytes — far larger than any
# delivered total in a capstone cell (<~1 MB). The period-256 pattern by itself
# is BLIND to a re-delivered batch whose length is a multiple of 256 (the
# duplicate re-aligns to the same phase and silently passes), which is exactly
# the "double-delivery inflating B/min" false-accept STEP 2c must catch. The
# block-index bytes destroy that symmetry: ANY hole, reorder, or duplicate — of
# ANY size — lands the wrong block index at some boundary and is caught as a
# pattern mismatch. Delivery over the mercury data socket is reliable + in-order
# (TCP-like), so the delivered stream is a contiguous prefix of this canonical
# sequence; a first_mismatch()!=None is therefore a genuine content/uniqueness
# violation, and the delivered-vs-fed count gate (rx<=tx, enforced in the
# runner) is the second, independent uniqueness assertion.
# ==========================================================================
_PAT = bytes(range(256))
CHUNK_LEN = 2048                           # == the original harness TX chunk (8*256)


def canonical_bytes(offset, length):
    """The canonical delivered-stream bytes at absolute [offset, offset+length).
    byte[k] = k%256 for k%256 not in {0,1,2}; the three block-boundary bytes carry
    the little-endian 24-bit block index (k//256) so the sequence is non-periodic
    (period 256*2**24). One source of truth for BOTH the TX generator and the RX
    verifier."""
    if length <= 0:
        return b""
    end = offset + length
    start_mod = offset % 256
    reps = (length + start_mod) // 256 + 2
    buf = bytearray((_PAT * reps)[start_mod:start_mod + length])
    first_block = offset // 256
    last_block = (end - 1) // 256
    for blk in range(first_block, last_block + 1):
        for c in range(3):                 # block-offsets 0,1,2 carry the block index
            pos = blk * 256 + c
            if offset <= pos < end:
                buf[pos - offset] = (blk >> (8 * c)) & 0xFF
    return bytes(buf)


def tx_chunk_at(offset, length=CHUNK_LEN):
    """The exact TX chunk the canonical ruler streams starting at absolute
    `offset` (non-periodic; see canonical_bytes)."""
    return canonical_bytes(offset, length)


def tx_chunk():
    """Back-compat: the first CHUNK_LEN bytes of the canonical stream (offset 0).
    New callers should use tx_chunk_at(offset) so the stream stays non-periodic."""
    return canonical_bytes(0, CHUNK_LEN)


def expected_slice(base_offset, length):
    """The bytes the delivered stream MUST contain at absolute [base, base+len)."""
    return canonical_bytes(base_offset, length)


def first_mismatch(base_offset, data):
    """Return the absolute offset of the first byte in `data` that deviates from
    the expected canonical stream, or None if the whole segment matches."""
    exp = canonical_bytes(base_offset, len(data))
    if data == exp:
        return None
    for j in range(len(data)):
        if data[j] != exp[j]:
            return base_offset + j
    return None


# ==========================================================================
# (b2) TWO-TRAFFIC SOURCES — compressible Winlink corpus vs incompressible random
# --------------------------------------------------------------------------
# The capstone scores DELIVERED throughput on TWO traffic classes, and the byte
# stream Mercury is fed MUST match the class so its compressor behaves as it
# would in the field. The old harness fed the SAME fixed ramp (tx_chunk_at) to
# BOTH classes and only applied a post-hoc x2.0907 to the pg84 score — a fiction,
# because (1) mercury ran `-F off` so the PPMd/zstd/dict streaming compressor
# never armed on the (non-B2F) ramp, and (2) applying the LZHUF ratio to the
# Mercury WIRE ramp while the VARA client bar is ALSO wire*2.0907 made the ratio
# CANCEL (secret wire-vs-wire), while the random arm compared Mercury WIRE against
# the VARA CLIENT bar (wire*2.0907) — a DOUBLE penalty. Both are fixed here:
#
#   * COMPRESSIBLE ("pg84"/"winlink"): a deterministic Winlink radio-email corpus
#     (headers + plain-language body drawn from a fixed vocabulary). Fed to a
#     mercury run with `-F on` so the production compressor ACTUALLY ARMS
#     (arq_commander.cc:6699 force_compress -> compression_enabled ->
#     streaming_enable(): PPMd order-6 + zstd L3 + Winlink dict priming). Because
#     the RX side DECOMPRESSES before it delivers (arq_common.cc:13784
#     decompress_block -> fifo_push_rx), the bytes the harness reads back ARE the
#     ORIGINAL decompressed content — so delivered B/min is the real
#     decompressed-content rate. NO synthetic multiplier: Mercury did the actual
#     compression, so its own ratio (PPMd+dict, not LZHUF) is already baked into
#     the delivered content. Scored vs the VARA CLIENT bar (VARA on-air wire *
#     per-corpus MEASURED LZHUF ratio — not a blind 2.0907).
#
#   * INCOMPRESSIBLE ("random-binary"): keyed counter-mode SHA-256 bytes that
#     neither LZHUF nor PPMd/zstd can shrink. Fed to a `-F off` run; delivered
#     bytes == wire bytes == content, so delivered B/min IS Mercury's wire rate.
#     Scored vs the VARA WIRE bar (LZHUF no-ops -> wire==content on BOTH sides;
#     removes the double penalty).
#
# BOTH sources are PURE FUNCTIONS OF THE ABSOLUTE OFFSET (a growable, cached,
# deterministic buffer). The TX generator and the RX byte-verifier both live in
# ONE harness process and share the cache, so they read identical bytes with no
# cross-process handshake; and BOTH are NON-REPEATING (every Winlink message and
# every 32-byte random block is distinct) so a period-aligned double-delivery
# still lands wrong bytes and is caught — the same uniqueness guarantee the old
# non-periodic ramp gave. Determinism is also cross-run stable (Python's
# Mersenne-Twister random + SHA-256), so measure_corpus_lzhuf_ratio() below
# re-generates the identical corpus offline to measure the LZHUF ratio.
# ==========================================================================
import hashlib as _hashlib
import random as _random

COMPRESSIBLE_TRAFFICS = ("pg84", "winlink", "text", "email")


def is_compressible_traffic(traffic):
    """True for the corpus/compressible class (pg84/winlink/text/email), False for
    random-binary/incompressible. Drives BOTH the fed source AND the VARA bar."""
    t = (traffic or "").lower()
    return any(t.startswith(x) for x in COMPRESSIBLE_TRAFFICS)


# ---- compressible: deterministic Winlink radio-email corpus ---------------
# A small fixed vocabulary of Winlink/ICS/ARES-flavoured tokens so the text is
# genuinely radio-email-like (exercises the firmware Winlink dict priming) AND
# compressible, while each generated message is unique (seeded on its index).
_WL_CALLS = ["W1AW", "K4ABC", "N0XYZ", "KD2QRS", "AA9DEF", "VE3GHI", "W7JKL",
             "KC5MNO", "N4PQR", "WB6STU", "K9VWX", "AC2YZA", "KE8BCD", "W5EFG"]
_WL_NAMES = ["John", "Mary", "Robert", "Linda", "David", "Susan", "James",
             "Karen", "Thomas", "Nancy", "Charles", "Betty", "Daniel", "Helen"]
_WL_PLACES = ["Springfield", "Riverton", "Fairview", "Bristol", "Clinton",
              "Georgetown", "Madison", "Salem", "Auburn", "Kingston", "Ashland"]
_WL_SUBJECTS = [
    "SITREP", "WELFARE CHECK", "SUPPLY REQUEST", "NET CONTROL LOG",
    "ICS-213 GENERAL MESSAGE", "SHELTER STATUS", "ROAD CONDITIONS",
    "POWER OUTAGE REPORT", "MEDICAL RESOURCE REQUEST", "WEATHER OBSERVATION",
    "TRAFFIC HANDLING", "STATION CHECK-IN", "DAMAGE ASSESSMENT",
]
# Word-level pools + templates. Sentences are ASSEMBLED (not picked from a small
# fixed set) so the body has realistic combinatorial variety — a 16-sentence
# corpus was pathologically repetitive (zlib ~14x), which would let mercury's
# STREAMING model exploit the repetition and beat VARA's per-message LZHUF for
# the WRONG reason. Word-level assembly over a ~150-token vocabulary lands the
# corpus in the realistic English range (PPMd ~2.5-4x, per-message LZHUF ~1.9x).
_WL_ADJ = ["priority", "urgent", "routine", "immediate", "stable", "critical",
           "partial", "full", "limited", "additional", "available", "affected",
           "damaged", "operational", "reduced", "increasing", "nominal", "severe",
           "moderate", "minor", "ongoing", "confirmed", "unconfirmed", "adequate"]
_WL_NOUN = ["shelter", "generator", "repeater", "supply", "convoy", "team",
            "coordinator", "net", "station", "sector", "bridge", "hospital",
            "clinic", "water", "fuel", "battery", "antenna", "operator",
            "message", "traffic", "outage", "casualty", "resource", "checkpoint",
            "roadway", "district", "county", "shelter site", "command post",
            "relay point", "power grid", "aid station", "staging area"]
_WL_VERB = ["reports", "requests", "confirms", "advises", "relays", "acknowledges",
            "deploys", "restores", "evacuates", "monitors", "establishes",
            "coordinates", "assesses", "distributes", "secures", "reroutes",
            "dispatches", "stages", "resupplies", "verifies"]
_WL_TIME = ["at zero six hundred", "by noon", "within the hour", "overnight",
            "at first light", "before dusk", "at the next net cycle", "immediately",
            "by end of shift", "at the top of the hour", "as soon as able"]
_WL_TAIL = ["due to flooding across the low ground", "pending fuel resupply",
            "and standing by for instructions", "with no injuries to report",
            "as conditions permit", "per the incident action plan",
            "until further notice", "and awaiting confirmation",
            "for immediate relay to net control", "once the road is cleared",
            "subject to weather", "and will update on the next cycle"]

_WL_BUF = bytearray()
_WL_MSG_IDX = 0


_GRID = "ABCDEFGHIJKLMNOPQR"


def _wl_specific(r):
    """A high-entropy realistic Winlink specific (grid/coords/qty/freq/time). Real
    SITREPs/ICS-213s carry lots of these unique numeric facts — they add the
    per-message entropy that keeps the streaming compressor honest (a book-like
    ~3-4x, not a rigged 10x from a tiny repeating vocabulary)."""
    k = r.randint(0, 5)
    if k == 0:
        return "grid %c%c%02d%c%c" % (r.choice(_GRID), r.choice(_GRID),
                                      r.randint(0, 99), chr(97 + r.randint(0, 23)),
                                      chr(97 + r.randint(0, 23)))
    if k == 1:
        return "at %02d.%04dN %02d.%04dW" % (r.randint(24, 49), r.randint(0, 9999),
                                             r.randint(66, 125), r.randint(0, 9999))
    if k == 2:
        return "qty %d of %d units" % (r.randint(1, 999), r.randint(1000, 9999))
    if k == 3:
        return "on %d.%03d MHz" % (r.randint(3, 29), r.randint(0, 999))
    if k == 4:
        return "ETA %02d%02d local" % (r.randint(0, 23), r.randint(0, 59))
    return "ref %06d" % r.randint(0, 999999)


def _wl_sentence(r):
    """Assemble one varied plain-language sentence from the word pools, often with
    a high-entropy specific appended (realistic + keeps entropy up)."""
    forms = [
        "%s %s %s the %s %s %s.",         # place team verbs the adj noun tail
        "The %s %s %s %s %s %s.",         # the adj noun verbs place-ish...
        "Requesting %s %s for the %s %s %s.",
        "%s %s and %s %s.",
    ]
    f = r.choice(forms)
    if f.startswith("The "):
        s = f % (r.choice(_WL_ADJ), r.choice(_WL_NOUN), r.choice(_WL_VERB),
                 r.choice(_WL_PLACES), r.choice(_WL_TIME), r.choice(_WL_TAIL))
    elif f.startswith("Requesting"):
        s = f % (r.choice(_WL_ADJ), r.choice(_WL_NOUN), r.choice(_WL_ADJ),
                 r.choice(_WL_NOUN), r.choice(_WL_TAIL))
    elif f.count("%s") == 4:
        s = f % (r.choice(_WL_PLACES), r.choice(_WL_VERB),
                 r.choice(_WL_VERB), r.choice(_WL_TAIL))
    else:
        s = f % (r.choice(_WL_PLACES), r.choice(_WL_NOUN), r.choice(_WL_VERB),
                 r.choice(_WL_ADJ), r.choice(_WL_NOUN), r.choice(_WL_TIME))
    if r.random() < 0.6:                    # 60% of sentences carry a unique specific
        s = s[:-1] + ", " + _wl_specific(r) + "."
    return s


def _wl_message(i):
    """Deterministic Winlink-style message #i (bytes). Realistic FBB/B2F-ish
    headers + an assembled plain-language body; seeded on i so every message is
    distinct (and the corpus is non-repeating over the whole run)."""
    r = _random.Random((0x57414C4B ^ (i * 2654435761)) & 0xFFFFFFFF)
    frm = r.choice(_WL_CALLS)
    to = r.choice(_WL_CALLS)
    op = r.choice(_WL_NAMES)
    place = r.choice(_WL_PLACES)
    subj = r.choice(_WL_SUBJECTS)
    # A B2F-like message id: 12 base-something chars.
    mid = "".join(r.choice("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789") for _ in range(12))
    yr, mo, da = 2026, r.randint(1, 12), r.randint(1, 28)
    hh, mm = r.randint(0, 23), r.randint(0, 59)
    nlines = r.randint(4, 10)
    lines = [
        "Mid: %s" % mid,
        "Date: %04d/%02d/%02d %02d:%02d" % (yr, mo, da, hh, mm),
        "Type: Private",
        "From: %s" % frm,
        "To: %s" % to,
        "Subject: %s DE %s AT %s" % (subj, frm, place),
        "Mbo: %s" % r.choice(_WL_CALLS),
        "Body: %d" % nlines,
        "",
    ]
    for _ in range(nlines):
        # 1-3 assembled sentences per line for varied line lengths.
        s = " ".join(_wl_sentence(r) for _ in range(r.randint(1, 3)))
        lines.append(s)
    lines.append("73 DE %s (%s)" % (frm, op))
    lines.append("/EX")
    lines.append("")
    return ("\r\n".join(lines)).encode("ascii", "replace")


# Optional REAL-corpus override. Point MERCURY_CAPSTONE_CORPUS at a real text
# corpus (e.g. Project Gutenberg pg84, or a concatenation of real Winlink B2F
# message bodies) for the DEFINITIVE beat-VARA verdict — real prose has a growing
# lexicon (Heaps' law) so it compresses at the documented realistic rates
# (PPMd+dict ~3.69x bulk text, LZHUF ~2.09x), whereas the built-in synthetic
# generator's fixed vocabulary lets the streaming model over-fit (measured ~6-8x
# PPMd / 1.76x LZHUF on 200 KB — fine for the SMOKE and self-contained CI, but on
# the compressible side of a real workload use the override). When the override is
# set, set MERCURY_CAPSTONE_LZHUF_RATIO to that corpus's measured LZHUF ratio
# (measure_corpus_lzhuf_ratio / the lzhuf_b2f CLI) so the VARA client bar matches.
_WL_FILE_BYTES = None    # None=unresolved; b""=use generator; non-empty=real corpus


def _wl_resolve_override():
    global _WL_FILE_BYTES
    import os
    p = os.environ.get("MERCURY_CAPSTONE_CORPUS")
    if p and os.path.isfile(p):
        try:
            with open(p, "rb") as f:
                _WL_FILE_BYTES = f.read()
            return
        except OSError:
            pass
    _WL_FILE_BYTES = b""


def _wl_ensure(n):
    """Grow the cached Winlink corpus to >= n bytes. Uses the real-corpus override
    file if MERCURY_CAPSTONE_CORPUS is set+readable (cycled to fill), else the
    deterministic synthetic Winlink generator."""
    global _WL_MSG_IDX, _WL_FILE_BYTES
    if _WL_FILE_BYTES is None:
        _wl_resolve_override()
    if _WL_FILE_BYTES:                       # real-corpus override (cycled)
        while len(_WL_BUF) < n:
            _WL_BUF.extend(_WL_FILE_BYTES)
        return
    while len(_WL_BUF) < n:                   # synthetic Winlink generator
        _WL_BUF.extend(_wl_message(_WL_MSG_IDX))
        _WL_MSG_IDX += 1


def winlink_slice(offset, length):
    """Bytes [offset, offset+length) of the deterministic Winlink corpus."""
    if length <= 0:
        return b""
    _wl_ensure(offset + length)
    return bytes(_WL_BUF[offset:offset + length])


# ---- incompressible: keyed counter-mode SHA-256 bytes ---------------------
_RB_KEY = b"mercury-capstone-incompressible-v1"
_RB_BUF = bytearray()


def _rb_ensure(n):
    """Grow the cached random buffer to >= n bytes (32-byte SHA-256 blocks)."""
    while len(_RB_BUF) < n:
        ctr = len(_RB_BUF) // 32
        _RB_BUF.extend(_hashlib.sha256(_RB_KEY + ctr.to_bytes(8, "little")).digest())


def random_slice(offset, length):
    """Bytes [offset, offset+length) of the deterministic incompressible stream."""
    if length <= 0:
        return b""
    _rb_ensure(offset + length)
    return bytes(_RB_BUF[offset:offset + length])


# ---- traffic dispatch + RX verifier (traffic-aware) -----------------------
def traffic_slice(traffic, offset, length):
    """The exact bytes the TX ruler streams at absolute `offset` for this traffic
    class (Winlink corpus for compressible, keyed-random for incompressible)."""
    if is_compressible_traffic(traffic):
        return winlink_slice(offset, length)
    return random_slice(offset, length)


def traffic_expected_slice(traffic, base_offset, length):
    """The bytes the delivered stream MUST contain at [base, base+len) — for
    compressible traffic these are the ORIGINAL (pre-compression) corpus bytes,
    which is exactly what mercury delivers after RX-side decompression."""
    return traffic_slice(traffic, base_offset, length)


def traffic_first_mismatch(traffic, base_offset, data):
    """First absolute offset in `data` deviating from the expected traffic stream,
    or None if the whole segment matches."""
    exp = traffic_slice(traffic, base_offset, len(data))
    if data == exp:
        return None
    for j in range(len(data)):
        if data[j] != exp[j]:
            return base_offset + j
    return None


# ==========================================================================
# (c) ANATOMY LINE-ITEMS
# --------------------------------------------------------------------------
# compute_inburst() is ported VERBATIM from tools/run_inband_ab.py (the proven
# anatomy profiler) so we WIRE the existing profiler in rather than re-derive.
# MARKERS are the datalink line-item regexes; a cell greps its combined log for
# these counts (retx rounds, demote events, BREAKs, SET_CONFIG tier-crosses,
# reverse-ACK RX-TIMEOUTs, in-band activity proof).
# ==========================================================================
# --------------------------------------------------------------------------
# CONFIG_NET_BPS: authoritative per-config NET PAYLOAD bit-rate, verbatim from
# `mercury.exe -l` (build 913ec8b5). This is the honest back-to-back frame rate
# (frame_payload_bits / total_frame_airtime, i.e. INCLUSIVE of that frame's
# preamble/pilot/guard overhead), NOT the during-data-symbol rate. Use it to
# derive the TRUE datalink efficiency = delivered_bps / net_bps(config). cfg16 =
# 3823.3 bps is the wire ceiling a pinned-cfg16 datalink can carry; VARA's wire
# is 6241.2 bps, so cfg16 is only 0.613x VARA on the wire (the dominant gap).
# --------------------------------------------------------------------------
CONFIG_NET_BPS = {
    0: 62.531017, 1: 136.972705, 2: 211.414392, 3: 285.856079, 4: 360.297767,
    5: 434.739454, 6: 583.622829, 7: 669.124424, 8: 807.373272, 9: 1083.870968,
    10: 1128.387097, 11: 1515.483871, 12: 1913.364055, 13: 2289.677419,
    14: 2676.774194, 15: 3348.387097, 16: 3823.325062, 17: 4451.612903,
}


def config_net_bps(config):
    """NET payload bps for an OFDM config id (0..17); None for ROBUST/unknown."""
    return CONFIG_NET_BPS.get(config)


def config_wire_cap_Bmin(config):
    """Physical WIRE CAP in bytes/min for an OFDM config id = net_bps/8*60. This is
    the absolute ceiling any honest throughput number for a datalink pinned/climbed
    to `config` can carry on the wire; None for ROBUST/unknown. cfg16 -> 28,675
    B/min, cfg17 -> 33,387 B/min. A reported rate above this is physically
    impossible (the C3/C4 109k-class artifact = decompressed-content mislabeled
    'wire')."""
    net = config_net_bps(config)
    return round(net / 8.0 * 60.0, 1) if net else None


def max_wire_cap_Bmin(configs):
    """The physical wire cap of the FASTEST WB OFDM config present in a run's
    configs-seen list (ROBUST ids 100/101/102 are excluded — they are the MFSK
    floor, not in the net-bps table). This is the tightest honest ceiling for the
    run: nothing the datalink keyed on air can exceed it. None if only ROBUST/
    unknown configs were seen. Used by the impossibility guard (C4)."""
    caps = [config_wire_cap_Bmin(c) for c in (configs or []) if c < 100]
    caps = [c for c in caps if c is not None]
    return max(caps) if caps else None


def flag_impossible_rate(rate_Bmin, cap_Bmin, tol=1.02):
    """True iff a throughput rate EXCEEDS the physical config wire cap by more than
    a 2% net_bps rounding margin (a metric bug — e.g. a decompressed-content rate
    mislabeled 'wire', which produced the 109,409 B/min artifact). Returns False
    when either input is unknown. One-line guard so a future 109k-class number is
    caught at the source (arq_realaudio.py) instead of propagating into a claim."""
    if rate_Bmin is None or cap_Bmin is None:
        return False
    return rate_Bmin > cap_Bmin * tol


# --------------------------------------------------------------------------
# GEOMETRY GUARD (metric-correction 2026-07-12). CONFIG_NET_BPS above is a HARDCODED
# table captured from `mercury.exe -l` at the STOCK frame geometry (runtime
# guard-interval Ngi=36, stock pilot Dy/Nsymb). EVERY table-derived wire meter --
# config_net_bps, datalink_efficiency, true_wire_keyed_Bmin, config_wire_cap_Bmin --
# is therefore BLIND to a per-run geometry override (the CP/pilot-reclaim escape
# hatch driven by MERCURY_SE_{NGI,DY,NSYMB}, or any future reclaim rung). Under such
# an override the table reports the STOCK rate (cfg15 3348.4 B... i.e. 3348.4 bps)
# while the real config was ~3826.7 bps (+14.3%), so datalink_eff_* OVER-reads and
# true_wire_keyed_Bmin UNDER-reads -- the reason prior analysis had to compute
# the airtime-normalised goodput.
#
# We do NOT recompute net_bps from the raw log geometry (that would replicate
# mercury's frame-airtime accounting = a fresh metric-bug risk). Instead we DETECT
# the override and (a) FLAG the table-derived wire meters untrustworthy for that run,
# (b) expose the geometry-INDEPENDENT trusted metrics (delivered_Bmin,
# bytes_per_fwd_airtime). Stock runs are unaffected and keep their table meters.
# --------------------------------------------------------------------------
STOCK_NGI = 36        # runtime guard-interval Ngi the CONFIG_NET_BPS table was captured at
                      # (verified across the retained corpus: 100% of Guard-interval prints
                      # are Ngi=36). A different value in a log => a CP/geometry override.
_GI_NGI_RE = re.compile(r"Guard interval:[^()]*\(Ngi=(\d+)")


def parse_runtime_ngi(logpath):
    """Return the sorted set of distinct runtime guard-interval Ngi values mercury
    printed (`Guard interval: X ms (Ngi=N, ...)`), or [] if none logged. Stock is
    [36]; any other value means a CP/geometry override was active for that run."""
    seen = set()
    try:
        with open(logpath, "r", errors="replace") as f:
            for line in f:
                m = _GI_NGI_RE.search(line)
                if m:
                    seen.add(int(m.group(1)))
    except OSError:
        pass
    return sorted(seen)


def geometry_override_flags(logpath, injected_env=None):
    """Decide whether the table-derived wire meters are trustworthy for THIS run.

    Two independent tells of a geometry override (EITHER trips it):
      * an injected env var whose name starts with MERCURY_SE_ (the CP/pilot-reclaim
        escape hatch that rewrites frame geometry);
      * a runtime guard-interval Ngi different from STOCK_NGI in the log.
    Returns {runtime_ngi:[...], stock_ngi, se_env_injected, geometry_is_stock,
    net_bps_geometry_blind}. geometry_is_stock True => CONFIG_NET_BPS is valid for
    this run; net_bps_geometry_blind True => DO NOT trust the table-derived wire
    meters (net_bps/datalink_eff_*/true_wire_keyed_Bmin/wire_cap_Bmin); use
    delivered_Bmin + bytes_per_fwd_airtime instead."""
    ngis = parse_runtime_ngi(logpath)
    se_env = False
    for kv in (injected_env or []):
        name = kv.split("=", 1)[0].strip()
        if name.startswith("MERCURY_SE_"):
            se_env = True
            break
    ngi_override = any(n != STOCK_NGI for n in ngis)
    is_stock = (not se_env) and (not ngi_override)
    return {
        "runtime_ngi": ngis,
        "stock_ngi": STOCK_NGI,
        "se_env_injected": se_env,
        "geometry_is_stock": is_stock,
        "net_bps_geometry_blind": not is_stock,
    }


def bytes_per_fwd_airtime(delivered_bytes, fwd_airtime_s):
    """GEOMETRY-INDEPENDENT airtime-normalised goodput: delivered payload bytes per
    second of forward PTT-on airtime. Needs NO net_bps table (pure measurement: real
    bytes / real keyed seconds), so it stays TRUSTWORTHY under a geometry override
    where the table-derived wire meters go blind. This is the honest number previously
    computed by hand. Returns None if the byte or airtime accounting is missing."""
    if not delivered_bytes or not fwd_airtime_s:
        return None
    return round(delivered_bytes / fwd_airtime_s, 2)


# ==========================================================================
# GEOMETRY FIRE-PROOF (pilot lattice) + TOP-GEAR ELECTION FIRE-PROOF.
# The Ngi-based geometry guard above (geometry_override_flags) detects a CP/guard-
# interval override, but the landing-stack cfg16 grid (Dy=5, Nsymb=8) changes the
# PILOT lattice, NOT Ngi -- so a cell that was SUPPOSED to run the thin grid but ran
# stock dense pilots reads geometry_is_stock=True and passes silently. That silently-
# STOCK cell is exactly what made the instrument read like stock. These parsers make
# the RUNTIME pilot geometry durable per cell (requires MERCURY_PILOT_DIAG=1, which
# only prints a diagnostic and changes no wire bytes) so a mis-geometried cell is
# DETECTABLE, and record the cfg17 election so a cfg17 rung cannot silently be cfg16.
#   telecom_system.cc:6454   [PILOT_DIAG] cfg%d Nc=.. Nsymb=.. Dx=.. Dy=.. nPilots=.. nData=.. ...
#   arq_commander.cc:1517    [TOPGEAR] queue SET_CONFIG: 16 -> 17 (<reason>; ...)
# ==========================================================================
_PILOT_DIAG_RE = re.compile(
    r"\[PILOT_DIAG\]\s+cfg(\d+)\s+Nc=(\d+)\s+Nsymb=(\d+)\s+Dx=(\d+)\s+Dy=(\d+)"
    r"\s+nPilots=(\d+)\s+nData=(\d+)")
_TOPGEAR_ELECT_RE = re.compile(
    r"\[TOPGEAR\]\s+queue SET_CONFIG:\s+(\d+)\s+->\s+(\d+)\s+\(")


def parse_pilot_geometry(logpath):
    """Parse the modem's [PILOT_DIAG] fire-proof lines into the LAST-observed runtime
    pilot lattice per config. Returns
      {"by_config": {cfg:{Nc,Nsymb,Dx,Dy,nPilots,nData}}, "matched_rows": N}
    matched_rows is printed next to any geometry statistic so a parser that silently
    matched ZERO lines (a stock binary, or MERCURY_PILOT_DIAG unset) cannot fabricate a
    tidy 'stock' answer -- matched_rows==0 means the fire-proof is ABSENT, not stock."""
    by_config = {}
    matched = 0
    try:
        with open(logpath, "r", errors="replace") as f:
            for raw in f:
                m = _PILOT_DIAG_RE.search(raw)
                if not m:
                    continue
                matched += 1
                cfg = int(m.group(1))
                by_config[cfg] = {
                    "Nc": int(m.group(2)), "Nsymb": int(m.group(3)),
                    "Dx": int(m.group(4)), "Dy": int(m.group(5)),
                    "nPilots": int(m.group(6)), "nData": int(m.group(7)),
                }
    except OSError:
        pass
    return {"by_config": by_config, "matched_rows": matched}


def parse_topgear_election(logpath):
    """Parse the cfg17 top-gear election fire-proof. Returns
      {"elected_16_to_17": bool, "transitions": [(from,to), ...], "matched_rows": N}
    elected_16_to_17 is the durable proof a cfg17 cell ACTUALLY climbed cfg16->cfg17
    (MERCURY_TOPGEAR_ELECT armed the sole cfg17 producer) instead of silently running
    cfg16. matched_rows==0 => no election line at all (feature unset or never engaged)."""
    transitions = []
    try:
        with open(logpath, "r", errors="replace") as f:
            for raw in f:
                m = _TOPGEAR_ELECT_RE.search(raw)
                if m:
                    transitions.append((int(m.group(1)), int(m.group(2))))
    except OSError:
        pass
    return {
        "elected_16_to_17": any(a == 16 and b == 17 for (a, b) in transitions),
        "transitions": transitions,
        "matched_rows": len(transitions),
    }


def assert_cell_geometry(pilot_geom, config, expected):
    """FIRE-PROOF assertion: does a cell's RECORDED runtime pilot geometry match the
    geometry INTENDED for its rung? Flags the silently-STOCK cell that made the
    instrument read like stock.

    pilot_geom : the parse_pilot_geometry() dict recorded in the cell.
    config     : the pinned/target config for this cell.
    expected   : {config: {"Dy":d, "Nsymb":n}, ...} intended geometry map. A config
                 ABSENT from `expected` carries no pilot assertion (stock grid).
    Returns (ok, reason). ok=True when there is nothing to assert OR the recorded
    grid matches; ok=False (with a reason) when the fire-proof is missing for a rung
    that HAS an intended geometry, or the recorded grid differs from it."""
    want = (expected or {}).get(config)
    if want is None:
        return True, None
    by_config = (pilot_geom or {}).get("by_config") or {}
    # by_config keys are ints in-process but STRINGS after a JSON round-trip (the recorded
    # cell). Accept either so the audit reads the same fire-proof the harness wrote.
    got = by_config.get(config)
    if got is None:
        got = by_config.get(str(config))
    if got is None:
        return False, (f"cfg{config}: pilot geometry fire-proof MISSING "
                       f"(matched_rows={(pilot_geom or {}).get('matched_rows')}); "
                       f"cannot confirm intended Dy={want.get('Dy')}/Nsymb={want.get('Nsymb')}")
    for key in ("Dy", "Nsymb"):
        if key in want and got.get(key) != want[key]:
            return False, (f"cfg{config}: runtime pilot geometry {key}={got.get(key)} "
                           f"!= intended {key}={want[key]} (silently-STOCK cell)")
    return True, None


def datalink_efficiency(delivered_bytes, wall_s, connected_at_s, config):
    """The TRUE datalink efficiency = fraction of the config's WIRE capacity that
    the datalink actually delivered, net of ALL overhead (guard/ACK/turnaround/
    retx/preamble/connect). Two variants:
      eff_full   = delivered_bps(full wall, incl connect)   / net_bps(config)
      eff_steady = delivered_bps(post-connect window)       / net_bps(config)
    Returns (eff_full, eff_steady, net_bps) or (None,None,None) if config unknown.
    This is the metric to report as the datalink duty cycle -- NOT active_fraction
    (see compute_inburst below), which measures batch-landing RATE and reads ~9x
    too low because the RX byte counter is a per-batch step function."""
    net = config_net_bps(config)
    if not net or not delivered_bytes or not wall_s:
        return None, None, net
    eff_full = round((delivered_bytes * 8 / wall_s) / net, 4)
    ss_win = max(1e-9, wall_s - (connected_at_s or 0.0))
    eff_steady = round((delivered_bytes * 8 / ss_win) / net, 4)
    return eff_full, eff_steady, net


def compute_inburst(timeline):
    """BATCH-LANDING RATE, *not* a data-airtime fraction. A poll (1 Hz sample of
    the RX byte counter) is ACTIVE iff measured_bytes increased since the previous
    sample. Because Mercury hands decoded data up in whole-BATCH chunks, the RX
    counter is a STEP function that jumps ONCE per batch delivery, so:
        active_frac ~= (# 1-second buckets containing a batch hand-up) / seconds
                     ~= batch-delivery-rate-per-second   (e.g. 35 batches/399s=0.088)
    This is ~9x LOWER than the true datalink efficiency (a cfg16 batch's ~9s of
    airtime is credited only the 1s in which its counter jump was polled), and
    in_burst_bps is correspondingly ~9x TOO HIGH (a batch's bytes / 1s instead of
    /~9s). Their product ~= the correct delivered wall bps, so delivered B/min is
    unaffected -- but DO NOT read active_frac as duty cycle. Use
    datalink_efficiency() above for the honest data-airtime/duty figure.
    Returns (in_burst_bps, active_frac, active_polls, total_polls, batch_sizes).
    Ported verbatim from tools/run_inband_ab.py:compute_inburst."""
    if len(timeline) < 2:
        return None, None, 0, max(len(timeline) - 1, 0), []
    active_bytes = active_time = 0.0
    active_polls = 0
    sizes = []
    for i in range(1, len(timeline)):
        t0, b0 = timeline[i - 1]
        t1, b1 = timeline[i]
        dt, db = t1 - t0, b1 - b0
        if db > 0 and dt > 0:
            active_bytes += db
            active_time += dt
            active_polls += 1
            sizes.append(int(db))
    total = len(timeline) - 1
    ib = round(active_bytes * 8 / active_time, 1) if active_time > 0 else None
    af = round(active_polls / total, 3) if total else None
    return ib, af, active_polls, total, sizes


# ==========================================================================
# (c') TRUE PTT-AIRTIME DUTY + TRUE ACK-ARRIVAL  (metric-correction 2026-07-03)
# --------------------------------------------------------------------------
# THREE broken rulers are replaced/supplemented here. All three read the SAME
# combined per-cell log (arq_<tag>.log), whose every line the harness stamps
# `[T+SSSSS.sss] [CMD|RSP] <mercury stdout>` (log_output(), arq_realaudio.py:102),
# so the T+ prefix IS the wall seconds since t0 — usable directly as an event
# clock without a second timer.
#
# FIX 1 — ptt_duty (TRUE PTT-airtime duty). active_fraction (compute_inburst)
#   samples the RX byte counter at 1 Hz and calls a poll "active" iff it stepped;
#   because Mercury hands data up in whole-BATCH chunks the counter steps ONCE per
#   batch, so active_fraction == batch-landing rate (~0.088), NOT a duty. The TRUE
#   forward duty is the fraction of wall the COMMANDER'S PTT was keyed. Every
#   forward TX burst is bracketed in the log by
#       [CMD] [CMD-TX] CONFIG_<c> batch=<b> ...      (arq_common.cc:8740, PTT ON)
#       [CMD] [TX-END] frames_to_read=...            (arq_common.cc:9358, PTT OFF)
#   so ptt_duty = Σ(t[TX-END] − t[CMD-TX]) / wall. (The reverse ACK is a short
#   unbracketed MFSK burst — [RSP] [TX-ACK-PAT] Sending ACK pattern — counted, not
#   airtime-summed.) msg_tx_time (from [CMD-POST-TX]) is Mercury's own per-batch
#   airtime and is summed as a cross-check, but it is printed on only a SUBSET of
#   paths so the bracket sum is the authoritative measure.
#
# FIX 2 — ack_arrival_ms (TRUE reverse-ACK arrival). receiving_timer.start() fires
#   the instant the commander enters RECEIVING_ACKS_DATA (end-of-TX;
#   arq_commander.cc:1952), so every `arrival_ms=`/`elapsed=` the commander prints
#   on an ACK detection is the ms from end-of-TX to that ACK landing — the real
#   turnaround the T1 fixed-slot mis-modeled. Sites:
#       [CMD-MFSK-ACK-SACK] CLEAN|PARTIAL ... arrival_ms=N     (already in mercury)
#       [CMD-SACK-V2] decoded SACK_RSP ... arrival_ms=N        (already in mercury)
#       [CMD-COMPACT-CONFIRM] CLEAN ... arrival_ms=N           (already in mercury)
#       [CMD-ACK-PAT] Data ACK pattern detected! elapsed=Nms   (added: branch
#                                                metrics/ack-arrival-elapsed off monitor)
#
# FIX 3 — actual listen guard. listen_guard_ms_mean parsed the CONFIGURED
#   receiving_timeout BUDGET (a constant ~3524 ms) — the ms the commander is ALLOWED
#   to wait, not the ms it actually waited. The ACTUAL guard is the ack_arrival_ms
#   when an ACK landed, and the full budget only when the window truly timed out.
#   ptt_duty_actual_guard_ms below is the honest reverse-ACK wait.
# ==========================================================================

# T+ line prefix the harness stamps (arq_realaudio.py:102). Group 1 = wall seconds.
_TS_RE = re.compile(r"^\[T\+(\d+(?:\.\d+)?)\]\s+\[(CMD|RSP)\]\s+(.*)$")
_CMDTX_RE = re.compile(r"\[CMD-TX\]\s+CONFIG_(\d+)\s+batch=(\d+)\s+type=(\d+)")
_TXEND_RE = re.compile(r"\[TX-END\]")
_POSTTX_RE = re.compile(r"\[CMD-POST-TX\]\s+receiving_timeout=(\d+)ms\s+msg_tx_time=(\d+)ms\s+batch=(\d+)")
_REVACK_TX_RE = re.compile(r"\[TX-ACK-PAT\]\s+Sending ACK pattern")
# ACK-arrival: match arrival_ms=N OR elapsed=Nms on a DATA-ACK detect line only.
# (Control-ACK lines also carry elapsed= — those are the connect handshake, not a
# data-batch turnaround, so they are excluded by requiring a data-ACK context tag.)
_ACK_ARRIVAL_RE = re.compile(
    r"\[(?:CMD-MFSK-ACK-SACK|CMD-SACK-V2|CMD-COMPACT-CONFIRM|CMD-ACK-PAT)\][^\n]*?"
    r"(?:arrival_ms=(\d+)|elapsed=(\d+)ms)")
# Kind tag for the corpus (clean vs partial vs sack).
_ACK_KIND_RE = re.compile(r"\b(CLEAN|PARTIAL)\b")


def parse_ack_arrival(text):
    """Return (arrival_ms:int, kind:str|None) for a DATA-ACK detect line, else None.
    kind is 'clean'|'partial'|'sack'|'bare' for corpus bucketing. Excludes the
    control-ACK 'Control ACK for code=' line (connect handshake, not a turnaround)."""
    if "Control ACK for code=" in text:
        return None
    m = _ACK_ARRIVAL_RE.search(text)
    if not m:
        return None
    ms = int(m.group(1) or m.group(2))
    km = _ACK_KIND_RE.search(text)
    if km:
        kind = km.group(1).lower()
    elif "CMD-SACK-V2" in text:
        kind = "sack"
    elif "CMD-ACK-PAT] Data ACK pattern detected" in text:
        kind = "bare"
    else:
        kind = None
    return ms, kind


def compute_ptt_duty(logpath, wall_full_s, connected_at_s=None):
    """TRUE forward PTT-airtime duty from the combined cell log (FIX 1).

    Sums the commander's [CMD-TX]→[TX-END] bracket durations (physical PTT-on wall
    time) and divides by wall. Returns a dict (or None if the log has no bracketed
    TX — e.g. a ROBUST-only or truncated cell):
      ptt_duty            : Σ fwd airtime / full wall            (headline duty)
      ptt_duty_steady     : Σ fwd airtime(post-connect) / (wall − connect)
      fwd_airtime_s       : total commander PTT-on seconds
      n_tx_bursts         : number of forward TX bursts
      ptt_duty_msgtx      : Σ msg_tx_time / full wall            (mercury-airtime cross-check)
      rev_ack_sends       : count of reverse-ACK MFSK bursts (unbracketed, short)
      configs_tx          : sorted set of configs the commander transmitted at
    All times are on the harness T+ clock, directly comparable to `dwell`."""
    brackets = []              # (t_start, t_end, cfg, type)
    open_tx = None
    msgtx_ms_sum = 0
    rev_ack = 0
    configs_tx = set()
    t_first = t_last = None
    try:
        with open(logpath, "r", errors="replace") as f:
            for raw in f:
                m = _TS_RE.match(raw.rstrip("\n"))
                if not m:
                    continue
                t = float(m.group(1))
                lab, txt = m.group(2), m.group(3)
                if t_first is None:
                    t_first = t
                t_last = t
                if lab == "RSP":
                    if _REVACK_TX_RE.search(txt):
                        rev_ack += 1
                    continue
                # CMD label from here
                mc = _CMDTX_RE.search(txt)
                if mc:
                    open_tx = (t, int(mc.group(1)), int(mc.group(3)))
                    configs_tx.add(int(mc.group(1)))
                    continue
                if _TXEND_RE.search(txt) and open_tx is not None:
                    brackets.append((open_tx[0], t, open_tx[1], open_tx[2]))
                    open_tx = None
                    continue
                mp = _POSTTX_RE.search(txt)
                if mp:
                    msgtx_ms_sum += int(mp.group(2))
    except OSError:
        return None
    if not brackets or not wall_full_s:
        return None
    fwd_airtime = sum(e - s for s, e, c, ty in brackets)
    ss_airtime = fwd_airtime
    ss_wall = wall_full_s
    if connected_at_s:
        ss_airtime = sum(e - s for s, e, c, ty in brackets if s >= connected_at_s)
        ss_wall = max(1e-9, wall_full_s - connected_at_s)
    # Per-config forward PTT airtime (seconds keyed at each config). Feeds the TRUE
    # on-air wire rate: Σ airtime_c × net_bps(c) = the physical payload bits keyed
    # (C4 — a wire number derived from frame accounting, NOT from decompressed RX).
    airtime_by_config = {}
    for s, e, c, ty in brackets:
        airtime_by_config[c] = airtime_by_config.get(c, 0.0) + (e - s)
    airtime_by_config = {c: round(v, 2) for c, v in airtime_by_config.items()}
    return {
        "ptt_duty": round(fwd_airtime / wall_full_s, 3),
        "ptt_duty_steady": round(ss_airtime / ss_wall, 3),
        "fwd_airtime_s": round(fwd_airtime, 1),
        "n_tx_bursts": len(brackets),
        "ptt_duty_msgtx": round((msgtx_ms_sum / 1000.0) / wall_full_s, 3),
        "rev_ack_sends": rev_ack,
        "configs_tx": sorted(configs_tx),
        "airtime_by_config": airtime_by_config,
    }


def keyed_wire_bytes(ptt):
    """Physical on-air payload bytes actually KEYED, from FRAME ACCOUNTING (never
    from decompressed RX): Σ_config (forward PTT airtime at that config × the
    config's authoritative net_bps / 8). net_bps already includes each frame's
    preamble/pilot/guard, so airtime×net_bps is the net payload the datalink put on
    the air (INCLUDING retransmitted payload — this is a wire count, not a deduped
    goodput). ROBUST/MFSK airtime is skipped (not in the net-bps table; a small
    floor term). Returns (keyed_bytes:float, per_config:dict{cfg:bytes}) or
    (None, {}) when PTT/airtime accounting is unavailable."""
    if not ptt:
        return None, {}
    abc = ptt.get("airtime_by_config") or {}
    if not abc:
        return None, {}
    total = 0.0
    per = {}
    for cfg, air in abc.items():
        net = config_net_bps(int(cfg))
        if net is None:
            continue
        b = air * net / 8.0
        per[int(cfg)] = round(b, 1)
        total += b
    if not per:
        return None, {}
    return round(total, 1), per


def true_wire_keyed_Bmin(ptt, wall_s):
    """TRUE on-air wire rate in bytes/min = keyed_wire_bytes / wall × 60. Because it
    is airtime × net_bps, it is bounded by the fastest config's net_bps BY
    CONSTRUCTION and can NEVER exceed the wire cap — the opposite of the
    decompressed-content 'wire' number that produced 109,409 B/min. Returns
    (rate_Bmin, keyed_bytes) or (None, None)."""
    kb, _per = keyed_wire_bytes(ptt)
    if kb is None or not wall_s:
        return None, None
    return round(kb / wall_s * 60.0, 1), kb


def collect_ack_arrivals(logpath):
    """Parse ALL DATA-ACK arrival_ms/elapsed values from a cell log (FIX 2).

    Associates each arrival with the config + batch_size of the batch it ACKs
    (from the most-recent preceding [CMD-TX] config and [CMD-POST-TX] batch on the
    CMD label). Returns list of dicts {arrival_ms,kind,config,batch_size} plus the
    raw sorted arrival list. Used by the harness ack_arrival_ms anatomy field AND
    by the offline corpus harvester (build_ack_arrival_corpus.py)."""
    rows = []
    cur_cfg = None
    cur_bsize = None
    try:
        with open(logpath, "r", errors="replace") as f:
            for raw in f:
                m = _TS_RE.match(raw.rstrip("\n"))
                if not m:
                    continue
                lab, txt = m.group(2), m.group(3)
                if lab != "CMD":
                    continue
                mc = _CMDTX_RE.search(txt)
                if mc:
                    cur_cfg = int(mc.group(1))
                    continue
                mp = _POSTTX_RE.search(txt)
                if mp:
                    cur_bsize = int(mp.group(3))
                    continue
                pa = parse_ack_arrival(txt)
                if pa is not None:
                    ms, kind = pa
                    rows.append({"arrival_ms": ms, "kind": kind,
                                 "config": cur_cfg, "batch_size": cur_bsize})
    except OSError:
        return [], []
    return rows, sorted(r["arrival_ms"] for r in rows)


def ack_arrival_stats(arrivals_ms):
    """median / mean / p10 / p90 / max / n for a list of ack arrival ms, or None."""
    xs = sorted(x for x in arrivals_ms if x is not None)
    if not xs:
        return None
    n = len(xs)
    def pct(p):
        if n == 1:
            return xs[0]
        k = (n - 1) * p
        lo = int(k)
        hi = min(lo + 1, n - 1)
        return round(xs[lo] + (xs[hi] - xs[lo]) * (k - lo), 1)
    return {"median": round(statistics.median(xs), 1),
            "mean": round(statistics.mean(xs), 1),
            "p10": pct(0.10), "p90": pct(0.90),
            "min": xs[0], "max": xs[-1], "n": n}


# ==========================================================================
# (c'') C5 — cfg17 HELD-vs-DEMOTED + live measure_variance (2026-07-04)
# --------------------------------------------------------------------------
# The endpoint-floor sweep recorded only decode16=1.0 even for cfg17 cells (no
# decode17 / config-held field), so cfg17-held-vs-demoted-to-cfg16 was UNRESOLVED.
# These parsers make it durable per cell from the SAME combined log:
#   * parse_config_timeline  -> (t, config) transitions from load_configuration().
#   * config_held_verdict    -> did a pinned cell HOLD its target WB config, or
#                               DEMOTE below it (to cfg16 / ROBUST)? = decode17 vs
#                               decode16 answer.
#   * parse_measure_variance -> live noise-variance / recovered-SNR readout (the
#                               `SNR=<f> dB` decode print = 10log10(1/variance),
#                               telecom_system.cc:3433,5976; and any `nv=` diag).
# ==========================================================================
# The modem prints "[CFG] load_configuration(N) current=M" BEFORE it assigns N
# (arq_common.cc load_configuration): N is the config being loaded, M the one it
# leaves (-1 before the first load). The timeline keys on N.
_LOADCFG_RE = re.compile(r"load_configuration\((-?\d+)\)\s+current=(-?\d+)")
# measure_variance readouts: recovered SNR (` SNR=<f> dB`, on every decoded frame)
# and the raw noise-variance estimate (`nv=<f>`, printed by the SFO-GRID/DIAG/
# BIGBLOCK-WAV decode diagnostics). Either populates the field.
_SNR_RE = re.compile(r"\bSNR=(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*dB")
_NV_RE = re.compile(r"\bnv=(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)")


def parse_config_timeline(logpath):
    """List of (t_wall_s, config) from every `load_configuration(N) current=M`
    line (config transitions), using the harness T+ prefix as the clock. Returns []
    on error/empty. config = N, the config the modem transitioned INTO (M is the
    config it left)."""
    out = []
    try:
        with open(logpath, "r", errors="replace") as f:
            for raw in f:
                m = _TS_RE.match(raw.rstrip("\n"))
                if not m:
                    continue
                t = float(m.group(1))
                mc = _LOADCFG_RE.search(m.group(3))
                if mc:
                    out.append((t, int(mc.group(1))))
    except OSError:
        return []
    return out


def _config_rank(c):
    """SPEED rank for a config id: WB OFDM 0..17 keep their id; ROBUST/MFSK ids
    (>=100) rank BELOW all WB (they are the slow floor). So a drop from cfg17 to
    cfg16 OR to ROBUST both register as a demote below the target."""
    return c if c < 100 else (c - 1000)


def config_held_verdict(timeline, target_config, connected_at_s=None, pinned=False):
    """C5: did a PINNED cell HOLD its target WB config, or DEMOTE below it? This is
    the durable decode17-vs-decode16 signal the endpoint-floor sweep lacked.

    target_config : the pinned/attempted WB config (arq_realaudio --start-cfg when
                    <100; None for ROBUST/auto -> verdict not applicable).
    connected_at_s: post-connect floor on the SAME (t0/cold) clock as the timeline.
                    For a GEARSHIFT session it filters out the climb-through configs
                    the modem loads WHILE ramping up during connect.
    pinned        : True for a PINNED session (--no-gearshift + --start-cfg, no climb).
                    A pinned cell loads its target config ONCE, during warm-up, which
                    under --warm-start happens BEFORE the cold connect completes (~35 s
                    load vs ~44 s connect). The post-connect floor then filters that one
                    load out -> reached_target reads False -> config_held=False, which
                    wrongly rejected 18/18 byte-exact pinned cells. When pinned, drop the
                    floor so the warm-up load counts as reaching the target; the demote
                    check still runs on events AFTER first reaching target, so a genuine
                    pinned-session fallback (a forced demote) still fails held. GEARSHIFT
                    sessions are untouched (pinned=False keeps the connect-floor behavior).
    Returns a dict:
      target_config   : echo
      applicable      : target is a WB config 0..17 (else the held question is moot)
      reached_target  : the modem loaded the target config at least once (past the floor)
      config_held     : reached target AND never dropped to a lower-rate config after
                        first reaching it (True/False; None if not applicable)
      demoted_to      : the highest-rate config it fell back to after reaching target
                        (the concrete 'demoted to cfg16' answer; None if held)
      max_wb_config   : highest WB config seen past the floor
      wb_configs_after: sorted WB configs seen past the floor (full picture)
      floor_s         : the post-connect floor actually applied (None when pinned)
    """
    tc = target_config
    applicable = tc is not None and 0 <= tc < 100
    # A pinned cell's target config loads during warm-up (before the cold connect),
    # so it must be visible to reached_target; drop the connect floor for pinned cells.
    floor_s = None if pinned else connected_at_s
    seen_after = [(t, c) for (t, c) in timeline
                  if floor_s is None or t >= floor_s]
    wb_after = sorted({c for (_t, c) in seen_after if c < 100})
    res = {
        "target_config": tc,
        "applicable": applicable,
        "reached_target": False,
        "config_held": None,
        "demoted_to": None,
        "max_wb_config": (max(wb_after) if wb_after else None),
        "wb_configs_after": wb_after,
        "floor_s": floor_s,
        "pinned": bool(pinned),
    }
    if not applicable:
        return res
    first_idx = next((i for i, (_t, c) in enumerate(seen_after) if c == tc), None)
    res["reached_target"] = first_idx is not None
    if first_idx is None:
        # Never even reached the pinned target -> it ran at something else the whole
        # time (a demote by definition). demoted_to = the fastest rung it did run.
        res["config_held"] = False
        res["demoted_to"] = res["max_wb_config"]
        return res
    tr = _config_rank(tc)
    after = [c for (_t, c) in seen_after[first_idx + 1:]]
    demotes = [c for c in after if _config_rank(c) < tr]
    if demotes:
        res["config_held"] = False
        res["demoted_to"] = max(demotes, key=_config_rank)   # closest fallback (e.g. cfg16)
    else:
        res["config_held"] = True
    return res


def parse_measure_variance(logpath):
    """Live measure_variance / recovered-SNR readout for a cell (C5 / CV8). Harvests
    every `SNR=<f> dB` (recovered SNR = 10log10(1/variance), printed on each decoded
    frame, telecom_system.cc:3433/5976) and every raw `nv=<f>` (noise-variance
    estimate, printed by the decode diagnostics). Returns
      {measure_variance:{median,mean,last,min,max,n},
       recovered_snr_db:{median,mean,last,min,max,n},
       source:'nv'|'snr_derived'|None}
    or None if neither appears. When only recovered SNR is present, measure_variance
    is derived as 10^(-SNR/10) (the inverse of the print). This ties the recovered
    SNR (31-33 @WGN:40) to the SAME variance metric that once read 14.4 dB. NOTE: a
    raw `nv=` requires a mercury nv-diag flag (e.g. MERCURY_BIGBLOCK_RXPB_DIAG); the
    `SNR=<f> dB` recovered SNR is emitted on every decoded frame without a flag."""
    import math
    snrs = []
    nvs = []
    try:
        with open(logpath, "r", errors="replace") as f:
            for raw in f:
                for m in _SNR_RE.finditer(raw):
                    try:
                        v = float(m.group(1))
                    except ValueError:
                        continue
                    if v > -90.0:            # skip the -99.9 sentinel (no measurement)
                        snrs.append(v)
                for m in _NV_RE.finditer(raw):
                    try:
                        v = float(m.group(1))
                    except ValueError:
                        continue
                    if v > 0.0:
                        nvs.append(v)
    except OSError:
        return None
    if not snrs and not nvs:
        return None

    def _stats(xs):
        if not xs:
            return None
        return {"median": round(statistics.median(xs), 6),
                "mean": round(statistics.mean(xs), 6),
                "last": xs[-1], "min": min(xs), "max": max(xs), "n": len(xs)}

    snr_stats = _stats(snrs)
    if nvs:
        mv_stats = _stats(nvs)
        source = "nv"
    else:
        derived = [10.0 ** (-s / 10.0) for s in snrs]
        mv_stats = _stats(derived)
        source = "snr_derived"
    return {"measure_variance": mv_stats,
            "recovered_snr_db": snr_stats,
            "source": source}


# Datalink anatomy markers. Producers live on BOTH CMD and RSP; the per-cell
# runner already tails both mercury stdouts into ONE combined log, so a single
# pass over that log counts them. Patterns verified against mercury/source
# printf sites (arq_commander.cc).
MARKERS = {
    # demote/BREAK family (the demote-amplifier line-items)
    "break_block":  r"\[BREAK\] Block failure",                         # arq_commander.cc:5135
    "break_any":    r"\[BREAK\]",                                       # any BREAK printf
    # A bare "DEMOTE" substring also matches [RSP-V2-DEMOTE-REBASE], which fires on EVERY
    # config change -- including CLIMBS. That inflated demote_events (arm A: 5 reported,
    # 0 real) and inverted the sign of the central gearshift metric.
    # STILL NOT a demote count: "LOSSLESS DEMOTE: rolling cmd_batch_seq_id" is bsi bookkeeping
    # (the config does not move), and a real 16->15 demote appears ONLY as
    # "[GEARSHIFT] SET_CONFIG: forward=15". A substring cannot decide direction. Any real
    # demote count must compare CAPABILITY RANK across SET_CONFIG transitions --
    # ids are not ordered by capability (100/101/102 are the robust tiers, below WB 0..17).
    # This marker is retained ONLY as a coarse "something demote-ish was printed" signal.
    # Do not use it as a demote count; it does not measure demotion events.
    "demote_lines": r"dropping to ROBUST",                              # the ONLY substring-visible demote
    "robust_drop":  r"dropping to ROBUST",                             # a genuine demote to robust
    "lossless_roll": r"LOSSLESS DEMOTE",                               # bsi roll; config unchanged
    "config_rebase": r"DEMOTE-REBASE",                                 # any config change (up or down)
    "set_config":   r"\[GEARSHIFT\] SET_CONFIG: forward=",              # arq_commander.cc:1201 (legacy tier-cross)
    # retransmit rounds (one printf per retx round on the commander)
    "retx_round":   r"\[CMD-RETX\] Sending|\[CMD-V2-MIXBATCH-RETX\]",   # arq_commander.cc:1848 / 2190
    # REVERSE-ACK MISS (metric-correction 2026-07-03): the ONLY faithful count of a
    # commander reverse-ACK listen-window that expired WITHOUT an ACK — i.e. a real
    # turnaround miss. arq_commander.cc:4917 prints exactly ONE of these per missed
    # window (then forces PENDING_ACK->ACK_TIMED_OUT and retransmits). This tracks
    # loss (heavy-loss cell = 9, light = 2 across the batchsize captures). The OLD
    # pattern below matched the CONSTANT per-batch `receiving_timeout=NNNms` BUDGET
    # printf ([CMD-POST-TX] + [CMD-RX] Entering receive), which fires ~2x/batch on
    # EVERY batch regardless of whether an ACK landed — so the pre-fix rx_timeout_events
    # (=70 on the 35-batch arm-A cell) was 2x the batch count, NOT a timeout count.
    "rx_timeout":   r"\[CMD-ACK-PAT\] Timeout: no ACK detected",
    # responder-side receive timeouts (distinct event: RSP waiting for forward data)
    "rsp_rx_timeout": r"\[RSP-RX-TIMEOUT\]|\[RSP-TIMEOUT\]|\[RSP-V2-GAP-ABORT\]",
    # BROKEN-legacy: the constant per-batch receiving_timeout BUDGET line count (NOT a
    # timeout count). Kept ONLY so old cohorts remain comparable; do NOT read as timeouts.
    "rx_timeout_budget_lines_BROKEN": r"receiving_timeout=|recv_timeout=",
    # in-band redesign activity proof (0 on legacy / all-off arm)
    "inband_tag":   r"\[INBAND-TX\] CONFIG_TAG|\[INBAND-RX\] CONFIG_TAG follow|\[INBAND-TX\] UNILATERAL",
    "inband_nobreak": r"\[INBAND-NOBREAK\]",
}
_MARKER_RE = {k: re.compile(v) for k, v in MARKERS.items()}

# LISTEN-GUARD ms (DIRECT): mercury prints the CMD reverse-ACK listen window per
# batch as "[CMD-POST-TX] receiving_timeout=NNNms" (arq_commander.cc:1949,
# unconditional) and "[CMD-RX] Entering receive mode: ... recv_timeout=NNN"
# (arq_commander.cc:1653, under -v). Each value is the ms the commander SITS
# LISTENING for the reverse ACK before it may retransmit - i.e. the listen guard
# itself. We sum/mean these to get listen-guard ms without an idle-fraction
# estimate. (There is no single per-event "listen guard elapsed" marker; this
# receiving_timeout budget is the faithful proxy the campaign uses.)
LISTEN_WINDOW_RE = re.compile(r"receiving_timeout=(\d+)ms|recv_timeout=(\d+)\b")


def parse_listen_window(text):
    """Return the listen-window ms in a log line, or None."""
    m = LISTEN_WINDOW_RE.search(text)
    if not m:
        return None
    return int(m.group(1) or m.group(2))


import re as _re_rd

_SETCFG_RE = _re_rd.compile(r"\[(CMD|RSP)\] \[GEARSHIFT\] SET_CONFIG: forward=(\d+)")
_ROBUST_DROP_RE = _re_rd.compile(r"dropping to ROBUST")


def config_rank(cfg):
    """Capability order. Config ids are NOT ordered by capability: the robust MFSK tiers
    100/101/102 sit BELOW every wideband rung 0..17. So `config change 100->0` is a CLIMB."""
    return (cfg - 100) if cfg >= 100 else (3 + cfg)


def count_real_demotes(logpath):
    """A demote is a move to a LOWER-capability config.

    Grounded in the retained corpus (2,108,062 lines): only three line-forms mention DEMOTE
    or a robust drop --
      [RSP-V2-DEMOTE-REBASE] config change X->Y   fires on EVERY config change, incl. climbs
      [CFG16-HOLD] LOSSLESS DEMOTE: rolling ...   bsi bookkeeping; the config does not move
      [BREAK] ... dropping to ROBUST_0            a genuine demote
    A real config demote (e.g. 16->15) prints ONLY as `[GEARSHIFT] SET_CONFIG: forward=15` and
    produces NO rebase line, so any rebase- or substring-based count misses it.
    """
    per_side = {"CMD": [], "RSP": []}
    robust_drops = 0
    try:
        with open(logpath, "r", errors="replace") as f:
            for line in f:
                m = _SETCFG_RE.search(line)
                if m:
                    per_side[m.group(1)].append(int(m.group(2)))
                if _ROBUST_DROP_RE.search(line):
                    robust_drops += 1
    except OSError:
        return {"real_demotes": None, "climbs": None, "robust_drops": None, "trajectory": {}}

    demotes = climbs = 0
    traj = {}
    for side, seq in per_side.items():
        ded = [c for i, c in enumerate(seq) if i == 0 or c != seq[i - 1]]
        traj[side] = ded
        demotes += sum(1 for a, b in zip(ded, ded[1:]) if config_rank(b) < config_rank(a))
        climbs += sum(1 for a, b in zip(ded, ded[1:]) if config_rank(b) > config_rank(a))
    return {"real_demotes": demotes + robust_drops, "climbs": climbs,
            "robust_drops": robust_drops, "trajectory": traj}


def count_markers(logpath):
    """Single pass over a combined cell log; returns {marker: count}."""
    counts = {k: 0 for k in MARKERS}
    try:
        with open(logpath, "r", errors="replace") as f:
            for line in f:
                for k, rx in _MARKER_RE.items():
                    if rx.search(line):
                        counts[k] += 1
    except OSError:
        pass
    return counts


# RETX ROUNDS corroboration (anti-fabrication, 2026-07-12). retx_rounds is sourced
# from the `retx_round` MARKER above, which matches the commander's per-round printfs
# [CMD-RETX] Sending / [CMD-V2-MIXBATCH-RETX]. Reconciled against the retained corpus:
# the emitted retx_rounds == the raw round-line count on EVERY cell (0 on a clean run;
# 9/9/11/1/5 on lossy runs), and batch_seq_id ADVANCE is forward batch progress, NOT
# a retx (so a 0 on a clean climb 0..26 is CORRECT, not a bug). The real hazard is the
# standing parser-matches-nothing trap: if a future mercury build renames the retx
# printf, the marker silently returns 0 and every cell reads a CLEAN 0 retx. count_retx
# corroborates the round count with the requeued-frame total from the SAME lines, and
# test_meters asserts the marker still FIRES (nonzero) on a known-lossy corpus cell.
_RETX_ROUND_RE = re.compile(r"\[CMD-RETX\] Sending|\[CMD-V2-MIXBATCH-RETX\]")
_RETX_REQUEUE_RE = re.compile(r"\[CMD-V2-MIXBATCH-RETX\]\s+R=(\d+)")


def count_retx(logpath):
    """Return {retx_rounds, retx_frames_requeued}. retx_rounds = commander retransmit
    rounds (one printf per round, == the `retx_round` marker); retx_frames_requeued =
    Σ R=<n> requeued frames across the MIXBATCH-RETX rounds -- a second, independent
    number from the same lines so a silent divergence is visible in the res JSON."""
    rounds = frames = 0
    try:
        with open(logpath, "r", errors="replace") as f:
            for line in f:
                if _RETX_ROUND_RE.search(line):
                    rounds += 1
                mr = _RETX_REQUEUE_RE.search(line)
                if mr:
                    frames += int(mr.group(1))
    except OSError:
        pass
    return {"retx_rounds": rounds, "retx_frames_requeued": frames}


# --------------------------------------------------------------------------
# PER-LEVER ENGAGEMENT counts (capstone STEP 2a). The manifest binary emits ONE
# greppable line at process teardown (atexit -> mercury_engage::print_summary in
# include/common/engagement_telemetry.h): each merged reverse-path lever tick()s
# its counter when it FIRES. Both the CMD and the RSP process emit their own line
# (the counters are per-process; a lever fires on whichever side owns it —
# SUPER-ACK/compact-confirm/ACK-slot-clip on the commander, HARQ/TINTERP on the
# data receiver), so the runner SUMS the two lines into one per-cell total.
#
#   [ENGAGE-SUMMARY] superack_leaps_landed=N compact_confirm_ok=N compact_confirm_fail=N
#                    harq_attempted=N harq_succeeded=N ack_slot_hit=N ack_slot_clip=N
#                    tinterp_activations=N
#
# CAPTURE REQUIRES A GRACEFUL EXIT: atexit does NOT run under SIGKILL, so the
# runner MUST SIGTERM (mercury installs a graceful SIGTERM/SIGINT handler:
# main.cc install_termination_handlers) and wait for the clean return before it
# releases the card. A process that survives the grace invalidates the cell and
# requires explicit card recovery; the runner must not hard-kill it. A lever that
# stays 0 where a fire was expected is the silently-dead / merge-misaligned signal.
# --------------------------------------------------------------------------
ENGAGE_FIELDS = ["superack_leaps_landed", "compact_confirm_ok", "compact_confirm_fail",
                 "harq_attempted", "harq_succeeded", "ack_slot_hit", "ack_slot_clip",
                 "tinterp_activations"]
ENGAGE_RE = re.compile(
    r"\[ENGAGE-SUMMARY\]\s+superack_leaps_landed=(\d+)\s+compact_confirm_ok=(\d+)\s+"
    r"compact_confirm_fail=(\d+)\s+harq_attempted=(\d+)\s+harq_succeeded=(\d+)\s+"
    r"ack_slot_hit=(\d+)\s+ack_slot_clip=(\d+)\s+tinterp_activations=(\d+)")


def parse_engage(text):
    """Parse an [ENGAGE-SUMMARY] line -> {field:int}, or None if not present."""
    m = ENGAGE_RE.search(text)
    if not m:
        return None
    return {ENGAGE_FIELDS[i]: int(m.group(i + 1)) for i in range(len(ENGAGE_FIELDS))}


def empty_engage():
    return {f: 0 for f in ENGAGE_FIELDS}


def add_engage(acc, one):
    """Accumulate a parsed engage dict (from one process) into acc (in place)."""
    for f in ENGAGE_FIELDS:
        acc[f] = acc.get(f, 0) + int(one.get(f, 0))
    return acc


# ==========================================================================
# (d) VARA-BAR reference
# --------------------------------------------------------------------------
# Owner-canonical VARA HF 4.8.9 CLIENT-effective B/min by IONOS WGN dial. Basis:
# Muething 2025-11-17 Winlink worksheet (Sheet1 G/H/I; H ``S:N`` is the IONOS
# dial, not independently measured external SNR3k; see tools/muething_throughput.py).
# wire thruput * LZHUF 2.0907 (same compression applied symmetrically to BOTH
# sides). @WGN40 = 46809*2.0907 = 97864 B/min (owner-canonical). Measured points
# {-10,0,10,20,30,40}; {-5,5,15} log-linear interpolated. Source:
# _research/capstone/setup.json vara_client_bar and the VARA bar methodology
# notes. This is
# the CORRECTED client bar (the discredited 3.72x was a units error).
# ==========================================================================
LZHUF = 2.0907
VARA_CLIENT_BMIN_BY_IONOS_DIAL = {
    -10: 2237, -5: 3751, 0: 6289, 5: 15483, 10: 38118,
    15: 55337, 20: 80335, 30: 94006, 40: 97864,
}
VARA_WIRE_BMIN_BY_IONOS_DIAL = {  # POST-LZHUF on-air wire delivery
    # These are the bytes VARA actually pushes over the air — i.e. the message
    # AFTER LZHUF compression (Muething 2025-11-17 worksheet thruput_B/min ==
    # POST-LZHUF wire delivery; _research/capstone/setup.json vara_client_bar
    # "Worksheet thruput_B/min = POST-LZHUF wire delivery"). The CLIENT bar above
    # is this wire rate * the LZHUF ratio (= the ORIGINAL/decompressed content
    # rate the operator sees). The old "pre-LZHUF wire" label was WRONG: LZHUF has
    # ALREADY been applied on the wire; multiplying wire by LZHUF RECOVERS the
    # original content size, it does not "add" a second compression pass.
    # This WIRE bar is the correct VARA reference for INCOMPRESSIBLE traffic,
    # where LZHUF no-ops so wire == content on BOTH sides (no double-penalty).
    -10: 1070, 0: 3008, 10: 18232, 20: 38425, 30: 44964, 40: 46809,
}


def _vara_table_on_physical_snr3k(table_by_dial):
    """Move worksheet dial coordinates onto the measured external-SNR axis.

    Values are unchanged; only each row's x coordinate is calibrated.  The
    one-decimal key matches snr3k_from_cell() and the measurement resolution.
    """
    return {round(ionos_wgn_to_snr3k(float(dial)), 1): value
            for dial, value in table_by_dial.items()}


VARA_CLIENT_BMIN = _vara_table_on_physical_snr3k(
    VARA_CLIENT_BMIN_BY_IONOS_DIAL)
VARA_WIRE_BMIN = _vara_table_on_physical_snr3k(
    VARA_WIRE_BMIN_BY_IONOS_DIAL)
VARA_MEASURED_SNR3K = sorted(VARA_WIRE_BMIN)
VARA_SOURCE = ("Muething 2025-11-17 Winlink worksheet IONOS-dial rows remapped "
               "to measured external SNR3k; wire x LZHUF 2.0907; "
               "_research/capstone/setup.json vara_client_bar; "
               "MEMORY vara_bar_methodology / reference_vara_benchmark_data")


def snr3k_from_cell(cell):
    """Map a bridge ``WGN:N`` label to measured IONOS external SNR3k."""
    if not cell:
        return None
    m = re.match(r"\s*WGN:(-?\d+(?:\.\d+)?)\s*$", cell, re.IGNORECASE)
    if not m:
        return None
    return round(ionos_wgn_to_snr3k(float(m.group(1))), 1)


def vara_bar(snr3k):
    """VARA client-effective B/min on the measured external-SNR3k axis."""
    return _loglin_interp(VARA_CLIENT_BMIN, snr3k)


def _loglin_interp(table, snr3k):
    """Log-linear-in-throughput interpolation of a {snr3k: B/min} table, clamped
    at the ends. Shared by vara_bar (client) and vara_wire."""
    if snr3k is None:
        return None
    keys = sorted(table)
    if snr3k in table:
        return table[snr3k]
    if snr3k <= keys[0]:
        return table[keys[0]]
    if snr3k >= keys[-1]:
        return table[keys[-1]]
    lo = max(k for k in keys if k < snr3k)
    hi = min(k for k in keys if k > snr3k)
    import math
    y0, y1 = math.log(table[lo]), math.log(table[hi])
    frac = (snr3k - lo) / (hi - lo)
    return round(math.exp(y0 + frac * (y1 - y0)), 1)


def vara_wire(snr3k):
    """VARA POST-LZHUF on-air WIRE B/min at a given SNR3k (the incompressible-arm
    reference: LZHUF no-ops so wire == content). Grid points exact; else
    log-linear in throughput."""
    return _loglin_interp(VARA_WIRE_BMIN, snr3k)


# --------------------------------------------------------------------------
# Per-corpus MEASURED LZHUF ratio (the VARA client-bar multiplier for the
# COMPRESSIBLE arm). This is the ratio VARA's FBB/B2F LZHUF achieves on the
# EXACT deterministic Winlink corpus this harness feeds — measured, NOT the blind
# owner-canonical 2.0907 (that was pg84-bulk-text). Measured with mercury's own
# FBB LZHUF (_research/b2f_unroll_harness/lzhuf_b2f{,.exe}, b2f=1 — the literal
# blob VARA puts on the wire), compressing each generated message INDEPENDENTLY
# (VARA's per-B2F-message model, no cross-message carry) and aggregating
# ratio = sum(orig) / sum(lzh). Re-measure with measure_corpus_lzhuf_ratio()
# whenever the corpus vocabulary/templates change.
#
#   MEASUREMENT (N=200 messages, lzhuf_b2f.exe, b2f=1, per-message independent):
#     orig=272922 lzh=154773 -> ratio = 272922/154773 = 1.763
#   (vs the owner-canonical pg84-bulk-text 2.0907 — this corpus is many small
#    Winlink emails with high-entropy specifics, so its per-B2F-message LZHUF
#    ratio is a bit lower, consistent with the REAL small-message b2f corpus:
#    email1 1.52x / gettysburg 1.80x / batch_3msg 1.72x. The point is it is
#    MEASURED on the exact fed corpus, not the assumed 2.0907.) Overridable via
#    MERCURY_CAPSTONE_LZHUF_RATIO when a real-corpus override file is used.
#
# random-binary: LZHUF cannot shrink keyed-random bytes -> 1.0 (and the
# incompressible arm uses the WIRE bar anyway, so this is only a guard).
WINLINK_CORPUS_LZHUF_RATIO = 1.763  # MEASURED (272922/154773, N=200, lzhuf_b2f b2f=1)
RANDOM_LZHUF_RATIO = 1.0


def corpus_lzhuf_ratio(traffic):
    """The MEASURED LZHUF ratio for this traffic's corpus (compressible only).
    MERCURY_CAPSTONE_LZHUF_RATIO overrides the baked constant (set it to the real
    override corpus's measured ratio when MERCURY_CAPSTONE_CORPUS is used)."""
    if not is_compressible_traffic(traffic):
        return RANDOM_LZHUF_RATIO
    import os
    ov = os.environ.get("MERCURY_CAPSTONE_LZHUF_RATIO")
    if ov:
        try:
            return float(ov)
        except ValueError:
            pass
    return WINLINK_CORPUS_LZHUF_RATIO


def vara_bar_for_traffic(snr3k, traffic):
    """The CORRECT VARA reference bar for the traffic class. Returns
    (bar_Bmin, kind, lzhuf_ratio):

      * compressible -> ("client"): VARA on-air WIRE * per-corpus MEASURED LZHUF
        ratio = VARA's delivered ORIGINAL-content B/min on this corpus. Compared
        against Mercury's delivered DECOMPRESSED-content B/min (both = original
        content bytes/min; Mercury's own PPMd+dict ratio is already in its number,
        VARA's LZHUF ratio is in this bar — the honest two-sided compression race).

      * incompressible -> ("wire"): VARA on-air WIRE B/min directly (LZHUF no-ops,
        wire == content on BOTH sides). Compared against Mercury's WIRE B/min.
        This removes the old double penalty (Mercury wire vs VARA wire*2.0907)."""
    w = vara_wire(snr3k)
    if w is None:
        return None, None, None
    if is_compressible_traffic(traffic):
        r = corpus_lzhuf_ratio(traffic)
        return round(w * r, 1), "client", r
    return round(w, 1), "wire", 1.0


def measure_corpus_lzhuf_ratio(lzhuf_bin, n_messages=200, tmpdir=None):
    """MEASURE the per-corpus LZHUF ratio the way VARA's B2F does: LZHUF-encode
    each generated Winlink message INDEPENDENTLY (b2f=1) and aggregate
    sum(orig)/sum(lzh). `lzhuf_bin` is the lzhuf_b2f CLI
    (_research/b2f_unroll_harness/lzhuf_b2f[.exe]). Returns
    {ratio, orig_bytes, lzh_bytes, n_messages}. Offline reproducibility tool for
    the WINLINK_CORPUS_LZHUF_RATIO constant; NOT called on the hot path."""
    import os
    import subprocess
    import tempfile
    td = tmpdir or tempfile.mkdtemp(prefix="wl_lzhuf_")
    os.makedirs(td, exist_ok=True)
    tot_orig = tot_lzh = 0
    for i in range(n_messages):
        msg = _wl_message(i)
        pin = os.path.join(td, "m%05d.plain" % i)
        pout = os.path.join(td, "m%05d.lzh" % i)
        with open(pin, "wb") as f:
            f.write(msg)
        subprocess.run([lzhuf_bin, "e", pin, pout], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        tot_orig += len(msg)
        tot_lzh += os.path.getsize(pout)
    return {"ratio": round(tot_orig / tot_lzh, 4) if tot_lzh else None,
            "orig_bytes": tot_orig, "lzh_bytes": tot_lzh, "n_messages": n_messages}


def user_multiplier(traffic):
    """DEPRECATED (kept for back-compat). The old two-traffic scoring multiplied
    the Mercury WIRE ramp by this to FAKE a compression benefit; that was wrong on
    both arms (see the (b2) section header). Mercury now ACTUALLY compresses the
    compressible corpus (`-F on`), so the delivered bytes ARE the original content
    and no synthetic multiplier applies -> 1.0 for every traffic. Scoring picks
    the VARA bar via vara_bar_for_traffic() instead."""
    return 1.0


def median_spread(xs):
    """Median + [min,max] + n over a list (None if empty)."""
    xs = [x for x in xs if x is not None]
    if not xs:
        return None
    return {"median": round(statistics.median(xs), 1),
            "min": round(min(xs), 1), "max": round(max(xs), 1), "n": len(xs)}


def mean_std_spread(xs):
    """mean/std(sample)/min/max/median/n over a list (None if empty). This is the
    PRIMARY anatomy summary the capstone verdict cites (mean/σ/min/max at N>=8)."""
    xs = [x for x in xs if x is not None]
    if not xs:
        return None
    n = len(xs)
    sd = round(statistics.stdev(xs), 2) if n > 1 else 0.0
    return {"mean": round(statistics.mean(xs), 2), "std": sd,
            "min": round(min(xs), 2), "max": round(max(xs), 2),
            "median": round(statistics.median(xs), 2), "n": n}


# --------------------------------------------------------------------------
# BIMODAL-LOCK scoring (capstone STEP 2b). The RX/ACK levers (SUPER-ACK,
# reverse-pin, compact-confirm) move P(fast-lock) far more than they move the
# in-burst bps of an already-fast run, so a POOLED mean dilutes their effect. We
# classify each run fast vs slow by an active_fraction threshold and report
# P(fast) plus per-MODE delivered B/min + active_fraction separately.
#
# active_fraction (compute_inburst) = fraction of 1 Hz polls in which the
# delivered byte-count increased. A fast-lock run that climbed to WB and streams
# steadily sits high; a slow-lock run stuck in ROBUST / BREAK-thrash with sparse
# deliveries sits low. DEFAULT_FAST_AF is the split; the raw per-run
# active_fraction list is always emitted so the bimodality is inspectable and the
# scorer can re-bin at any threshold.
# --------------------------------------------------------------------------
DEFAULT_FAST_AF = 0.5


def bimodal_score(results, af_threshold=DEFAULT_FAST_AF):
    """Split a cohort's per-run result dicts into fast/slow by active_fraction and
    return P(fast) + per-mode {delivered_user_Bmin, active_fraction} mean/σ spreads."""
    def af(r):
        # active_fraction is BATCH-LANDING CADENCE, not a duty (see compute_inburst);
        # used HERE only to split fast/slow lock, which cadence legitimately separates.
        # Read the labelled key first (active_fraction_CADENCE_NOT_DUTY /
        # active_fraction_BROKEN); fall back to the legacy bare key for old cohort JSONs.
        an = r.get("anatomy") or {}
        v = an.get("active_fraction_CADENCE_NOT_DUTY")
        if v is None:
            v = an.get("active_fraction_BROKEN")
        if v is None:
            v = an.get("active_fraction")
        return v

    def ub(r):
        return (r.get("anatomy") or {}).get("delivered_user_Bmin")

    classified = [(r, af(r)) for r in results if af(r) is not None]
    n_cls = len(classified)
    fast = [r for r, a in classified if a >= af_threshold]
    slow = [r for r, a in classified if a < af_threshold]
    p_fast = round(len(fast) / n_cls, 3) if n_cls else None
    return {
        "af_threshold": af_threshold,
        "n_classified": n_cls,
        "n_fast": len(fast),
        "n_slow": len(slow),
        "p_fast": p_fast,
        "fast": {
            "delivered_user_Bmin": mean_std_spread([ub(r) for r in fast]),
            "active_fraction": mean_std_spread([af(r) for r in fast]),
        },
        "slow": {
            "delivered_user_Bmin": mean_std_spread([ub(r) for r in slow]),
            "active_fraction": mean_std_spread([af(r) for r in slow]),
        },
        "active_fraction_all": sorted(round(a, 3) for _, a in classified),
    }
