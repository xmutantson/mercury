#!/usr/bin/env python3
"""Unit contract for fixed DATA config -> CONNECT/clock harness policy."""

import sim_arq_channel as harness


def main():
    # Ordinary/adaptive runs remain byte-compatible: no env override, bare wire.
    assert harness.resolve_connect_policy(100, False, None) == (None, 0, False)
    assert harness.resolve_connect_policy(15, False, None) == (None, 0, False)

    # A pin is one config coordinate and gets shared-clock CONNECT by default.
    assert harness.resolve_connect_policy(14, True, None) == (14, 1, True)
    assert harness.resolve_connect_policy(15, True, None) == (15, 1, True)
    assert harness.resolve_connect_policy(16, True, None) == (16, 1, True)
    assert harness.resolve_connect_policy(101, True, None) == (101, 1, True)

    # Explicit wire choices remain exact for compatibility and diagnostic A/Bs.
    assert harness.resolve_connect_policy(15, True, 0) == (15, 0, False)
    assert harness.resolve_connect_policy(15, True, 1) == (15, 1, False)

    print("SIM pinned-connect policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
