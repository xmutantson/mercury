#!/usr/bin/env python3
"""test_sim_port_autopick.py — prove the sim_arq_channel.py port auto-pick is
collision-proof for CONCURRENT invocations.

Root cause this guards: the harness used to hardcode RSP=7002/CMD=7006 + relay
52100. A live sibling experiment holding those ports blocked three consecutive
calibration runs. pick_free_ports() now bind-probes a free quad and advances the
base on collision, so N concurrent sims never collide.

Run:  python tools/sim/test_sim_port_autopick.py
Exit 0 = all pass. No mercury/relay processes are spawned (port logic only).
"""
import socket
import sys

import sim_arq_channel as m


def _hold(ports):
    """Bind+listen real sockets on `ports` to simulate a concurrent run owning
    them. Returns the socket list (caller must keep them alive / close them)."""
    socks = []
    for p in ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # NO SO_REUSEADDR — we want a genuine exclusive hold, exactly like a live
        # peer modem, so the picker's probe must skip these ports.
        s.bind(("127.0.0.1", p))
        s.listen(1)
        socks.append(s)
    return socks


def _quad_set(picked):
    """The 4 ctrl/data ports + relay from a pick_free_ports() return tuple."""
    return set(picked)


def test_default_quad_when_free():
    """[T-DEFAULT] with nothing held, the first pick is the documented default
    quad (7002/7003/7006/7007 + 52100). Stable for deterministic re-runs."""
    r = m.pick_free_ports()
    assert r == (7002, 7003, 7006, 7007, 52100), r
    print("[T-DEFAULT] free machine -> default quad 7002/7003/7006/7007+52100  PASS")


def test_two_concurrent_invocations_non_colliding():
    """[T-CONCURRENT] the headline test. Invocation #1 picks a quad and HOLDS all
    five of its ports (modelling a live sim/bench run). Invocation #2 must pick a
    DISJOINT quad — no shared port at all."""
    a = m.pick_free_ports()
    held = _hold(a)            # invocation #1 now owns its 5 ports
    try:
        b = m.pick_free_ports()  # invocation #2 picks while #1 holds
        sa, sb = _quad_set(a), _quad_set(b)
        assert sa.isdisjoint(sb), f"quads collide: a={a} b={b} overlap={sa & sb}"
        # And the ctrl-base actually advanced by the stride (not just the relay).
        assert b[0] == a[0] + m.PORT_PICK_STRIDE, (a[0], b[0])
        print(f"[T-CONCURRENT] #1 holds {a} -> #2 picks DISJOINT {b}  PASS")
    finally:
        for s in held:
            s.close()


def test_three_way_non_colliding():
    """[T-THREEWAY] three concurrent runs all get pairwise-disjoint quads — the
    fleet-twoproc-grid prerequisite (more than two cells at once)."""
    a = m.pick_free_ports()
    ha = _hold(a)
    try:
        b = m.pick_free_ports()
        hb = _hold(b)
        try:
            c = m.pick_free_ports()
            sets = [_quad_set(a), _quad_set(b), _quad_set(c)]
            for i in range(3):
                for j in range(i + 1, 3):
                    assert sets[i].isdisjoint(sets[j]), \
                        f"collision between run {i} and {j}: {sets[i] & sets[j]}"
            print(f"[T-THREEWAY] three concurrent runs disjoint: {a[0]},{b[0]},{c[0]}  PASS")
        finally:
            for s in hb:
                s.close()
    finally:
        for s in ha:
            s.close()


def test_ctrl_base_override_pins_base():
    """[T-PIN] --ctrl-base pins the base for a deterministic re-run when free."""
    r = m.pick_free_ports(ctrl_base=7300)
    assert r[0] == 7300 and r[1] == 7301 and r[2] == 7304 and r[3] == 7305, r
    print(f"[T-PIN] ctrl_base=7300 -> quad {r[:4]}  PASS")


def test_pinned_base_busy_fails_loudly():
    """[T-PIN-BUSY] a PINNED base that is busy must FAIL LOUDLY (RuntimeError),
    not silently hand out a colliding quad — determinism beats silent drift."""
    held = _hold([7400])     # occupy just the ctrl port of the pinned base
    try:
        # tries=1 so it does not advance off the pinned base.
        try:
            m.pick_free_ports(ctrl_base=7400, tries=1)
        except RuntimeError:
            print("[T-PIN-BUSY] pinned-but-busy base raised RuntimeError  PASS")
            return
        raise AssertionError("pinned busy base did NOT raise")
    finally:
        for s in held:
            s.close()


def test_relay_avoids_quad():
    """[T-RELAY] when the relay preference lands inside the chosen ctrl/data quad,
    the relay is bumped off it (no relay==ctrl/data collision)."""
    # Pin a base, then prefer a relay port that sits inside that quad.
    r = m.pick_free_ports(ctrl_base=7500, relay_pref=7504)  # 7504 is CMD ctrl
    assert r[4] not in (r[0], r[1], r[2], r[3]), r
    print(f"[T-RELAY] relay {r[4]} avoids quad {r[:4]}  PASS")


def main():
    tests = [
        test_default_quad_when_free,
        test_two_concurrent_invocations_non_colliding,
        test_three_way_non_colliding,
        test_ctrl_base_override_pins_base,
        test_pinned_base_busy_fails_loudly,
        test_relay_avoids_quad,
    ]
    failed = 0
    for t in tests:
        try:
            t()
        except AssertionError as e:
            print(f"[FAIL] {t.__name__}: {e}")
            failed += 1
    if failed:
        print(f"\n{failed}/{len(tests)} FAILED")
        return 1
    print(f"\nALL {len(tests)} PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
