#!/usr/bin/env python3
"""Regression for the arq_realaudio.py configs_seen CFG_RE gap (BUG-A current= keying).

The per-cell driver records configs_seen from `[CFG] load_configuration(N) current=M`.
The bug keyed on group(2)=current=M (the OUTGOING/previous config) instead of
group(1)=N (the TARGET being loaded), so a freshly-adopted WB id (cfg0/13/16) never
appeared and configs_seen under-reported the climb / cfg0 cross. It also used
current=(\\d+) which dropped the current=-1 default-init line.

This drives CFG_RE over a real trio climb-and-cross log slice (ROBUST_0 100 ->101
->102 -> WB CONFIG_0) the way log_output() does, and asserts configs_seen reflects
the TARGETS actually loaded (incl cfg0), NOT the previous configs.

FAIL-BEFORE: with the pre-fix `add(int(m.group(2)))`, configs_seen would be the set
of `current=` values {-1?,100,101,102} and would NOT contain 0 -> the cfg0 assert
fails (the exact under-report this fixes).
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arq_realaudio as ra  # noqa: E402

# Real climb-and-cross log lines (trio 144001a4, WGN:40). Each load prints the
# TARGET N in load_configuration(N) and the PREVIOUS config in current=M.
LOG = [
    "[T+0002.015] [RSP] [CFG] load_configuration(100) current=-1 level=FULL backup=NO",
    "[T+0005.017] [CMD] [CFG] load_configuration(100) current=-1 level=FULL backup=NO",
    "[T+0086.463] [CMD] [CFG] load_configuration(101) current=100 level=PHYSICAL backup=YES",
    "[T+0088.437] [RSP] [CFG] load_configuration(101) current=100 level=PHYSICAL backup=YES",
    "[T+0095.993] [CMD] [CFG] load_configuration(102) current=101 level=PHYSICAL backup=YES",
    "[T+0156.080] [RSP] [CFG] load_configuration(102) current=101 level=PHYSICAL backup=YES",
    # THE CROSS: 102 (robust) -> CONFIG_0 (WB OFDM). Pre-fix keyed on current=102
    # and DROPPED the cfg0 target -> configs_seen missing 0 (the under-report).
    "[T+0163.249] [CMD] [CFG] load_configuration(0) current=102 level=PHYSICAL backup=YES",
    "[T+0165.482] [RSP] [CFG] load_configuration(0) current=102 level=PHYSICAL backup=YES",
]


def collect_targets(lines):
    seen = set()
    for line in lines:
        m = ra.CFG_RE.search(line)
        if m:
            seen.add(int(m.group(1)))  # mirrors the fixed log_output()
    return seen


def main():
    seen = collect_targets(LOG)

    # The regex must match the cfg0 load line AND the current=-1 default-init line.
    assert ra.CFG_RE.search(LOG[0]) is not None, \
        "CFG_RE must match current=-1 (default-init) — needs current=(-?\\d+)"
    assert ra.CFG_RE.search(LOG[-1]) is not None, "CFG_RE must match the cfg0 load line"

    # TARGET-keyed configs_seen must contain every config actually LOADED, incl the
    # WB cfg0 cross. (Pre-fix it would be {100,101,102} keyed on current=M, no 0.)
    expected = {100, 101, 102, 0}
    assert seen == expected, f"configs_seen target-keying wrong: got {sorted(seen)} expected {sorted(expected)}"

    # And it must NOT contain the spurious -1 (current= group is no longer the key).
    assert -1 not in seen, "configs_seen must key on TARGET N, never on current=M (-1 leaked in)"

    # Downstream climb signals (mirrors arq_realaudio result builder) must now read cfg0.
    wb_seen = sorted(c for c in seen if c < 100)
    assert wb_seen == [0], f"wb_configs_seen must report the cfg0 cross, got {wb_seen}"
    assert max(seen) == 102, "max robust rung preserved"

    print("[OK] CFG_RE target-keying: configs_seen=%s wb_seen=%s (cfg0 cross reported)"
          % (sorted(seen), wb_seen))
    return 0


if __name__ == "__main__":
    sys.exit(main())
