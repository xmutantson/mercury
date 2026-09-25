#!/usr/bin/env python3
"""Regression guard for the real-audio harness's config / demote meters.

Three meters were wrong at once, and together they seeded four campaign-level claims
that all turned out to be false:

  1. configs_seen keyed on `current=` -- the config being LEFT. The newest-adopted
     config is therefore NEVER recorded, because a run ends on it and it never appears
     as a `current=`. The regex also demanded `current=(\\d+)`, dropping `current=-1`.
     Symptom: a run that logged 465 `configuration:CONFIG_16` lines reported
     max_config_reached=14.

  2. max_config_reached took max() over a set mixing the robust MFSK tiers
     (100/101/102) with the wideband rungs (0..17). Config ids are NOT ordered by
     capability -- robust sits BELOW every WB rung -- so it returned 102 on every run
     that used robust, blind to the WB ceiling.

  3. demote_events was the bare substring `DEMOTE`, which matches
     `[RSP-V2-DEMOTE-REBASE] config change 13->14` (fires on EVERY config change,
     INCLUDING CLIMBS) and `[CFG16-HOLD] LOSSLESS DEMOTE: rolling cmd_batch_seq_id`
     (bsi bookkeeping; the config does not move). It also MISSED real demotes: a
     genuine 16->15 prints only as `[GEARSHIFT] SET_CONFIG: forward=15` and emits no
     rebase line at all.

Run: python tools/sim/realaudio/test_meters.py
Exit 0 = all meters honest. Nonzero = a meter is lying again.
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import arq_realaudio as ra          # noqa: E402
import capstone_arms as ca          # noqa: E402

WS = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
CORPUS = os.path.join(WS, "_research", "precook_ab_v2", "logs")

failures = []


def check(cond, msg):
    if cond:
        print("  PASS  %s" % msg)
    else:
        print("  FAIL  %s" % msg)
        failures.append(msg)


# ---------------------------------------------------------------- 1. configs_seen
print("configs_seen / CFG_RE")
check(ra.CFG_RE.search("[CFG] load_configuration(13) current=-1 level=FULL") is not None,
      "CFG_RE matches the current=-1 default-init line")

m = ra.CFG_RE.search("[CFG] load_configuration(16) current=13 level=PHYS_ONLY")
check(m is not None and int(m.group(1)) == 16,
      "CFG_RE group(1) is the TARGET being loaded (16), not the config being left (13)")

src = open(os.path.join(HERE, "arq_realaudio.py"), encoding="utf-8").read()
check("configs_seen.add(int(m.group(2)))" not in src,
      "configs_seen does NOT key on group(2) (the outgoing config)")

# ---------------------------------------------------------------- 2. max_wb ceiling
print("\nmax_config_reached / WB ceiling")
check("max_wb_config_reached" in src,
      "the cell summary exposes max_wb_config_reached (the real WB ceiling)")
check(ca.config_rank(0) > ca.config_rank(102),
      "capability rank: WB cfg0 outranks ROBUST_2 (ids are not ordered by capability)")
check(ca.config_rank(16) > ca.config_rank(15) > ca.config_rank(0),
      "capability rank is monotonic across the WB rungs")

# ---------------------------------------------------------------- 3. demote counting
print("\ndemote counting")
rx = re.compile(ca.MARKERS["demote_lines"])
check(not rx.search("[RSP] [RSP-V2-DEMOTE-REBASE] config change 13->14 mid-transfer"),
      "the demote marker does NOT match DEMOTE-REBASE (which fires on climbs too)")
check(not rx.search("[CMD] [CFG16-HOLD] FIX-9 LOSSLESS DEMOTE: rolling cmd_batch_seq_id 13"),
      "the demote marker does NOT match LOSSLESS DEMOTE (a bsi roll; config unchanged)")
check(bool(rx.search("[RSP] [BREAK] Responding with ACK, dropping to ROBUST_0")),
      "the demote marker DOES match a genuine robust drop")
check("demote" not in ca.MARKERS,
      "the old direction-blind 'demote' marker key is gone")

# ---------------------------------------------------------------- 4. ground truth
print("\ncount_real_demotes vs the retained corpus")
if not os.path.isdir(CORPUS):
    print("  SKIP  corpus not present at %s" % CORPUS)
else:
    # Hand-derived from the CMD SET_CONFIG trajectory, cross-checked against the
    # independent per-side `configuration:CONFIG_N` status field.
    expected = {"arq_A_C1_s1": 0, "arq_A_C1_s2": 0, "arq_A_C1_s3": 0,
                "arq_B_C1_s1": 0, "arq_B_C1_s2": 0, "arq_B_C1_s3": 2}
    for cell, want in sorted(expected.items()):
        p = os.path.join(CORPUS, cell + ".log")
        if not os.path.exists(p):
            print("  SKIP  %s absent" % cell)
            continue
        got = ca.count_real_demotes(p)["real_demotes"]
        check(got == want, "%s real_demotes=%d (expected %d)" % (cell, got, want))

    # The climbs must NOT be counted as demotes -- that was the original bug.
    r = ca.count_real_demotes(os.path.join(CORPUS, "arq_A_C1_s1.log"))
    check(r["climbs"] >= 4 and r["real_demotes"] == 0,
          "arq_A_C1_s1 climbs %d rungs and demotes 0 times (it was reported as 5 demotes)" % r["climbs"])

# ---------------------------------------------------------------- 5. retx_rounds
# retx_rounds reads 0 on a CLEAN cell (a 0..N batch climb is forward progress, NOT a
# retx) and == the raw [CMD-V2-MIXBATCH-RETX] round count on a lossy cell. The hazard
# is the parser-matches-nothing trap: a future printf rename would silently zero it on
# EVERY cell. Guard = positive fixture (marker still fires) + corpus reconciliation +
# a NONZERO assertion on a known-lossy cell (goes red if the marker ever stops matching).
print("retx_rounds (anti-fabrication)")
_retx_rx = re.compile(ca.MARKERS["retx_round"])
check(bool(_retx_rx.search("[CMD] [CMD-V2-MIXBATCH-RETX] R=5 (bsi= 18 18 18 18 18) of 5 total queued")),
      "retx_round marker MATCHES a canonical MIXBATCH-RETX line (not matches-nothing)")
check(bool(_retx_rx.search("[CMD] [CMD-RETX] Sending retransmit")),
      "retx_round marker also matches the legacy [CMD-RETX] Sending form")
if os.path.isdir(CORPUS):
    # (log basename, expected retx_rounds) hand-derived by grep -c on the raw log.
    retx_expect = {"arq_A_C1_s1": 0, "arq_A_C1_s2": 9, "arq_A_C2_s1": 11}
    for base, want in sorted(retx_expect.items()):
        p = os.path.join(CORPUS, base + ".log")
        if not os.path.exists(p):
            print("  SKIP  %s absent" % base)
            continue
        got = ca.count_retx(p)
        check(got["retx_rounds"] == want,
              "%s count_retx=%d (raw marker count %d)" % (base, got["retx_rounds"], want))
        if want > 0:
            check(got["retx_frames_requeued"] > 0,
                  "%s retx_frames_requeued>0 corroborates the %d rounds" % (base, want))
    # THE anti-fabrication assertion: a known-lossy cell MUST report nonzero retx.
    lossy = os.path.join(CORPUS, "arq_A_C2_s1.log")
    if os.path.exists(lossy):
        check(ca.count_retx(lossy)["retx_rounds"] > 0,
              "known-lossy cell arq_A_C2_s1 reports retx_rounds>0 (a 0 here = printf renamed, meter silently dead)")
else:
    print("  SKIP  corpus not present")

# ---------------------------------------------------------------- 6. geometry guard
# CONFIG_NET_BPS is a hardcoded STOCK-geometry table; every wire meter derived from it
# is BLIND to a per-run CP/pilot override. Guard = detect the override (Ngi != 36 OR a
# MERCURY_SE_* env var) and expose the geometry-INDEPENDENT trusted goodput.
print("\ngeometry guard (net_bps blindness)")
check(ca.STOCK_NGI == 36, "STOCK_NGI == 36 (the geometry CONFIG_NET_BPS was captured at)")
# CONSISTENCY ASSERTION: the table must be internally monotonic + match its wire cap.
_nb = [ca.CONFIG_NET_BPS[c] for c in range(0, 18)]
check(all(a < b for a, b in zip(_nb, _nb[1:])),
      "CONFIG_NET_BPS is monotonic across WB rungs 0..17 (self-consistent table)")
check(ca.config_wire_cap_Bmin(15) == round(ca.CONFIG_NET_BPS[15] / 8.0 * 60.0, 1),
      "config_wire_cap_Bmin(15) is consistent with the net_bps table")
# Override detection.
check(ca.geometry_override_flags("/nonexistent.log", ["MERCURY_SE_NGI=54"])["net_bps_geometry_blind"] is True,
      "a MERCURY_SE_* injected env is flagged net_bps_geometry_blind")
check(ca.geometry_override_flags("/nonexistent.log", [])["se_env_injected"] is False,
      "no SE env => se_env_injected False")
# Synthetic Ngi-override log -> flagged blind; stock corpus log -> stock.
import tempfile  # noqa: E402
_tmp = os.path.join(tempfile.gettempdir(), "test_meters_ngi_override.log")
with open(_tmp, "w") as _f:
    _f.write("[T+0002.009] [RSP] Guard interval: 4.50 ms (Ngi=54, gi=0.21)\n")
check(ca.parse_runtime_ngi(_tmp) == [54], "parse_runtime_ngi reads a non-stock Ngi=54 from the log")
check(ca.geometry_override_flags(_tmp, [])["net_bps_geometry_blind"] is True,
      "a runtime Ngi!=36 in the log is flagged net_bps_geometry_blind")
os.remove(_tmp)
check(ca.bytes_per_fwd_airtime(1000, 100.0) == 10.0,
      "bytes_per_fwd_airtime is a pure measurement (1000 B / 100 s = 10 B/s), geometry-independent")
check(ca.bytes_per_fwd_airtime(0, 100.0) is None and ca.bytes_per_fwd_airtime(1000, 0) is None,
      "bytes_per_fwd_airtime returns None on missing byte/airtime accounting")
if os.path.isdir(CORPUS):
    p = os.path.join(CORPUS, "arq_A_C1_s1.log")
    if os.path.exists(p):
        g = ca.geometry_override_flags(p, [])
        check(g["runtime_ngi"] == [36] and g["geometry_is_stock"] is True,
              "stock corpus cell reports runtime_ngi=[36], geometry_is_stock=True")

# ---------------------------------------------------------------- 7. active_fraction
# active_fraction is BATCH-LANDING CADENCE (~9x-deflated vs duty). The bare unlabelled
# key is retired; only the labelled active_fraction_CADENCE_NOT_DUTY + ptt_duty_steady
# (the real duty) are emitted. bimodal_score must read the labelled key.
print("\nactive_fraction (cadence, not duty)")
check('"active_fraction": active_frac' not in src,
      "the bare unlabelled 'active_fraction' key is NOT emitted in the anatomy dict")
check('"active_fraction_CADENCE_NOT_DUTY": active_frac' in src,
      "active_fraction is emitted ONLY under the label active_fraction_CADENCE_NOT_DUTY")
check('"ptt_duty_steady"' in src,
      "ptt_duty_steady (the TRUE forward duty, 0.79-0.82 measured) is emitted")
_fake = [{"anatomy": {"active_fraction_CADENCE_NOT_DUTY": 0.9, "delivered_user_Bmin": 100.0}},
         {"anatomy": {"active_fraction_CADENCE_NOT_DUTY": 0.1, "delivered_user_Bmin": 5.0}}]
_bm = ca.bimodal_score(_fake, af_threshold=0.5)
check(_bm["n_classified"] == 2 and _bm["n_fast"] == 1,
      "bimodal_score reads the labelled active_fraction_CADENCE_NOT_DUTY key (classified 2, 1 fast)")

# ---------------------------------------------------------------- 8. rx_timeout budget
# The constant per-batch receiving_timeout BUDGET line is NOT a timeout count; it is
# renamed *_BROKEN. The honest reverse-ACK miss count is the `rx_timeout` marker, which
# must actually match real miss lines (not matches-nothing).
print("\nrx_timeout budget vs real misses")
check("rx_timeout_budget_lines_BROKEN" in ca.MARKERS and "rx_timeout_budget" not in ca.MARKERS,
      "the budget-line count is labelled rx_timeout_budget_lines_BROKEN (no bare rx_timeout_budget)")
_rxto = re.compile(ca.MARKERS["rx_timeout"])
check(bool(_rxto.search("[CMD] [CMD-ACK-PAT] Timeout: no ACK detected, peak_matched=3/7")),
      "the real rx_timeout marker MATCHES a canonical miss line (not matches-nothing)")
_budget = re.compile(ca.MARKERS["rx_timeout_budget_lines_BROKEN"])
check(not _rxto.search("[CMD] [CMD-POST-TX] receiving_timeout=3524ms msg_tx_time=9000ms batch=1")
      and bool(_budget.search("[CMD] [CMD-POST-TX] receiving_timeout=3524ms")),
      "the constant budget line counts as _BROKEN, NOT as a real rx_timeout miss")
if os.path.isdir(CORPUS):
    p = os.path.join(CORPUS, "arq_A_C2_s1.log")
    if os.path.exists(p):
        check(ca.count_markers(p)["rx_timeout"] > 0,
              "known-lossy cell arq_A_C2_s1 reports rx_timeout>0 real misses (marker still fires)")

print()
if failures:
    print("[FAIL] %d meter check(s) failed:" % len(failures))
    for f in failures:
        print("   - %s" % f)
    sys.exit(1)
print("[OK] all config/demote/retx/geometry/duty/timeout meters are honest")
