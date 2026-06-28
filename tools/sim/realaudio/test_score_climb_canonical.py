#!/usr/bin/env python3
"""
test_score_climb_canonical.py - durable regression test for the canonical climb
scorer. Proves the two scorer-bug classes are dead and that the canonical scorer
reproduces the two re-validated fleet runs from their LITERAL raw RSP/CMD lines.

It runs the OLD buggy keying (current=M, group 2) side by side with the canonical
scorer on a single witness line, so the bug stays visible as a strike-through
(the OLD column must show the LIE, the NEW column the truth).

Corpus = verbatim raw lines preserved in
  _research/climbdiag/reconfirm.json        (run w0i0te56f-lineage, b5bb0e22)
  _research/climbdiag/verify_wb-throughput.json (run wejx9za7i, wb-throughput)
the time-zero artifacts. These are a regression FIXTURE so the scorer is checkable
now (fleet logs are transient) and re-checkable against the raw logs later.

Run:  python test_score_climb_canonical.py   (exit 0 = all pass)
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import score_climb_canonical as C  # the canonical (fixed) scorer

# ---- OLD buggy scorer: keyed on current=M (group 2); no RSP/CMD split, no gate.
#      Kept ONLY to assert it produces the LIE the canonical scorer fixes. ----
OLD_CFG_RE = re.compile(r"load_configuration\((\d+)\)\s+current=(-?\d+)")


def old_score(lines):
    configs_seen = set()
    for ln in lines:
        m = OLD_CFG_RE.search(ln)
        if m:
            configs_seen.add(int(m.group(2)))   # BUG: keys on current=M (outgoing)
    cs = sorted(configs_seen)
    wb = [c for c in cs if 0 <= c <= 16]
    return {"configs_seen": cs, "wb_configs_seen": wb}


# ============================ FIXTURES ============================
# (1) RECONFIRM run (w0i0te56f-lineage, reconfirm.json literal_lines): the RSP
#     loads CONFIG_0 (WB) and decodes WB DATA_LONG there at 66.4 bps -> cross=YES.
RECONFIRM_RSP_CROSS = [
    "[T+0143.302] [CMD] [CMD-ACK-PAT] Control ACK for code=59 detected! elapsed=1411ms link=2 status=3",
    "[T+0143.502] [CMD] [CFG] load_configuration(0) current=102 level=PHYS_ONLY backup=NO",
    "[T+0143.891] [CMD] [GEARSHIFT] SET_CONFIG ACKed, loaded config 0",
    "[T+0144.332] [CMD] [INBAND-TX] CONFIG_TAG passband emit cfg=0 (ladder_idx=3) bsi_lsb=3 parity=0 burst_samples=75920",
    "[T+0146.016] [RSP] [CFG] load_configuration(0) current=102 level=PHYS_ONLY backup=NO",
    "[T+0146.756] [RSP] [INBAND-RX] ROBUST-TIER tag-follow to CONFIG_0 (was robust); BREAK avoided",
    "[T+0149.835] [RSP] [RX-BATCH-SEQ] type=DATA_LONG id=1 seq=1 batch_seq_id=3 (v2) -> nReceived_data=4, on configuration:CONFIG_0 (66.4 bps)",
    "[T+0151.100] [RSP] [RX-BATCH-SEQ] type=DATA_LONG id=2 seq=2 batch_seq_id=4 (v2)",
    "[T+0152.400] [RSP] [RX-BATCH-SEQ] type=DATA_LONG id=3 seq=3 batch_seq_id=5 (v2)",
    "[T+0188.000] [RSP] stats.nReceived_data= 13",
]

# (2) WB-THROUGHPUT run (wejx9za7i, verify_wb-throughput.json): the RSP crosses to
#     CONFIG_0 and decodes MANY WB DATA frames there, but NEVER load_configuration
#     to cfg1..16 -> crossed=YES, climbed_past_cfg0=False. Synthesize one cell
#     (wb_monitor_s35_p0: 80 wb frames on cfg0, max_wb_cfg_with_data=0, rxB 697).
WBTHRU_RSP_STALL = (
    [
        "[T+0043.710] [CMD] link_status:Connected to TESTB",
        "[T+0043.710] [RSP] link_status:Connected to TESTA",
        "[T+0150.700] [RSP] [CFG] load_configuration(0) current=102 level=PHYS_ONLY backup=NO",
    ]
    # 80 WB DATA frames on CONFIG_0, no further load_configuration -> no climb past 0
    + [f"[T+{151.0 + i:08.3f}] [RSP] [RX-BATCH-SEQ] type=DATA_LONG id={i} seq={i} (v2)"
       for i in range(80)]
    + ["[T+0560.000] [RSP] stats.nReceived_data= 83"]
)

# (3) LEGACY / no-cross (CROSSING_GROUNDTRUTH trio-OFF): RSP trajectory
#     100->101->BREAK->100, never a WB id, any DATA is on ROBUST -> cross=NO.
LEGACY_NOCROSS = [
    "[T+0001.052] [RSP] [CFG] load_configuration(100) current=-1 level=FULL backup=NO",
    "[T+0050.000] [RSP] [RX-BATCH-SEQ] type=DATA_LONG id=0 seq=0 batch_seq_id=0 (v2)",
    "[T+0111.000] [CMD] [CFG] load_configuration(101) current=100 level=PHYS_ONLY backup=NO",
    "[T+0121.000] [CMD] [GEARSHIFT] SET_CONFIG ACKed, loaded config 101",
    "[T+0140.000] [RSP] [BREAK] Block failure",
    "[T+0145.000] [RSP] [CFG] load_configuration(100) current=101 level=PHYS_ONLY backup=NO",
]

# (4) PARSE-BUG WITNESS: one RSP WB load line. OLD must record 102 (current=M, the
#     LIE -> wb_configs_seen=[]); canonical must record TARGET 0.
WITNESS = ["[T+0146.016] [RSP] [CFG] load_configuration(0) current=102 level=PHYS_ONLY backup=NO"]

# (5) A WB climb-PAST-cfg0 fixture (forward-looking; not yet seen on the bench):
#     RSP loads cfg0, decodes, then loads cfg2 and decodes -> climbed_past_cfg0=True.
PAST_CFG0 = [
    "[T+0150.000] [RSP] [CFG] load_configuration(0) current=102 level=PHYS_ONLY backup=NO",
    "[T+0151.000] [RSP] [RX-BATCH-SEQ] type=DATA_LONG id=1 seq=1 (v2)",
    "[T+0160.000] [RSP] [CFG] load_configuration(2) current=0 level=PHYS_ONLY backup=NO",
    "[T+0161.000] [RSP] [RX-BATCH-SEQ] type=DATA_LONG id=2 seq=2 (v2)",
]


def score(lines, rx_bytes=None):
    rsp, cmd, breaks = C.score_lines(lines)
    return C.build_result(rsp, cmd, breaks, "fixture", rx_bytes)


def main():
    ok = True
    checks = []

    # --- (1) RECONFIRM: cross to cfg0 is REAL, decodes, ~66 bps ---
    r = score(RECONFIRM_RSP_CROSS, rx_bytes=125)
    o = old_score(RECONFIRM_RSP_CROSS)
    checks += [
        ("reconfirm canonical crossed==True", r["crossed"] is True),
        ("reconfirm canonical rsp_loaded_wb_id==0", r["rsp_loaded_wb_id"] == 0),
        ("reconfirm canonical rsp_wb_data_frames==3", r["rsp_wb_data_frames"] == 3),
        ("reconfirm canonical time_to_cross_s==149.835", r["time_to_cross_s"] == 149.835),
        ("reconfirm canonical climbed_past_cfg0==False", r["climbed_past_cfg0"] is False),
        ("reconfirm OLD wb_configs_seen==[] (the BUG-A lie)", o["wb_configs_seen"] == []),
    ]

    # --- (2) WB-THROUGHPUT: cross to cfg0, 80 frames, but climb STALLS at cfg0 ---
    r = score(WBTHRU_RSP_STALL, rx_bytes=697)
    checks += [
        ("wbthru canonical crossed==True", r["crossed"] is True),
        ("wbthru canonical rsp_wb_data_frames==80", r["rsp_wb_data_frames"] == 80),
        ("wbthru canonical max_wb_cfg_with_data==0", r["max_wb_cfg_with_data"] == 0),
        ("wbthru canonical climbed_past_cfg0==False", r["climbed_past_cfg0"] is False),
        ("wbthru canonical rsp_nreceived_frames==83", r["rsp_nreceived_frames"] == 83),
    ]

    # --- (3) LEGACY: no WB load on RSP -> no cross ---
    r = score(LEGACY_NOCROSS)
    checks += [
        ("legacy canonical crossed==False", r["crossed"] is False),
        ("legacy canonical rsp_loaded_wb_id is None", r["rsp_loaded_wb_id"] is None),
    ]

    # --- (4) WITNESS: OLD=102 (bug), canonical=target 0 ---
    r = score(WITNESS)
    o = old_score(WITNESS)
    checks += [
        ("witness OLD records 102 not 0 (BUG-A)", o["configs_seen"] == [102] and o["wb_configs_seen"] == []),
        ("witness canonical records target 0", r["rsp_loaded_wb_id"] == 0 and r["wb_configs_seen"] == [0]),
    ]

    # --- (5) bytes-gate: cross PHY true but rx_bytes==0 -> crossed False ---
    r0 = score(RECONFIRM_RSP_CROSS, rx_bytes=0)
    checks += [("bytes-gate rx_bytes==0 -> crossed==False", r0["crossed"] is False)]

    # --- (6) climb PAST cfg0 detected when it happens ---
    r = score(PAST_CFG0, rx_bytes=2048)
    checks += [
        ("past-cfg0 canonical max_wb_cfg_with_data==2", r["max_wb_cfg_with_data"] == 2),
        ("past-cfg0 canonical climbed_past_cfg0==True", r["climbed_past_cfg0"] is True),
    ]

    for name, cond in checks:
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}")
        ok = ok and cond
    print("\nRESULT:", "ALL PASS" if ok else "FAILURES PRESENT")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
