#!/usr/bin/env python3
"""Summarize the Watterson envelope sweep JSON: per-layout net-PHY + envelope
tables (block-success rate vs depth x Doppler), and the FINAL-layout pick."""
import json, sys

PASS_THRESH = 0.875   # >=7/8 blocks decode = "holds" (the gearshift-handoff edge)

def load(p):
    with open(p) as f: return json.load(f)

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "bigblock_hw/watterson_envelope.json"
    R = load(path)
    P = R["params"]
    depths, dopplers = P["depths"], P["dopplers"]
    print(f"# Watterson slow/mild-fade envelope  (seeds/cell={P['seeds']}, "
          f"EsN0={P['esn0']}, VARA={P['vara']})")
    print(f"# block_airtime(incl-pre)={P['block_airtime_s']:.3f}s   "
          f"'holds' = >= {PASS_THRESH:.0%} of seed-blocks decode 8/8\n")

    layout_summary = []
    for lkey, L in R["layouts"].items():
        nphy = L.get("net_phy_data_payload")
        nphy_pre = L.get("net_phy_incl_preamble")
        pil = L.get("pilots_pct"); K = L.get("K")
        gate = "PASS" if (nphy and nphy > P["vara"]) else "FAIL"
        print(f"## LAYOUT {lkey}  pilots={pil}%  K={K}  "
              f"net-PHY={nphy:.0f} (payload) / {nphy_pre:.0f} (incl-pre) bps  "
              f"vs VARA {P['vara']}: {gate}")
        # envelope grid
        hdr = "depth\\fd | " + " | ".join(f"{fd:>6}" for fd in dopplers)
        print(hdr)
        # find the deepest depth that HOLDS at each Doppler
        holds_at = {fd: -1 for fd in dopplers}
        for depth in depths:
            cells = []
            for fd in dopplers:
                c = L["cells"].get(f"{depth}|{fd}", {})
                r = c.get("block_ok_rate", 0.0)
                cells.append(f"{r:>5.0%}")
                if r >= PASS_THRESH and depth > holds_at[fd]:
                    holds_at[fd] = depth
            print(f"{depth:>8} | " + " | ".join(f"{c:>6}" for c in cells))
        env = ", ".join(f"fd{fd}:<= {holds_at[fd]}dB" if holds_at[fd] >= 0 else f"fd{fd}:NONE"
                        for fd in dopplers)
        print(f"  envelope (deepest holding depth per Doppler): {env}\n")
        layout_summary.append((lkey, nphy, nphy_pre, pil, K, gate, holds_at))

    print("\n# ===== SUMMARY =====")
    print(f"{'layout':>8} {'pil%':>6} {'K':>3} {'netPHY':>7} {'gate':>5}  envelope(deepest-hold dB per fd)")
    for lkey, nphy, nphy_pre, pil, K, gate, holds_at in layout_summary:
        env = " ".join(f"{fd}:{holds_at[fd]}" for fd in dopplers)
        print(f"{lkey:>8} {pil:>6} {K:>3} {nphy:>7.0f} {gate:>5}  {env}")

if __name__ == "__main__":
    main()
