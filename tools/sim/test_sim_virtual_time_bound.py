#!/usr/bin/env python3
"""
test_sim_virtual_time_bound.py — assert the SIM ARQ harness run is bounded by
VIRTUAL time / completion, NOT by real wall-clock seconds.

WHY THIS TEST EXISTS
--------------------
The modem core clock is VIRTUAL (sim_clock.cc:43-52: virtual_ns = samples *
1e9/48000; advanced ONLY by sim_clock_add_samples(len) per forwarded chunk in
audioio.c rx_transfer:1753-1754). Every climb-path deadline (gearshift
receiving_timeout, PTT turnaround, the conservative-PDES barrier) reads this
virtual clock, NEVER the wall clock. So the ROBUST_0->CFG16 climb trajectory is a
function of VIRTUAL time only and MUST be host-load-independent.

The one place that broke that property was sim_arq_channel.py's monitor loop,
which bounded the whole run by `time.time() - start < args.secs` (REAL seconds).
Under host CPU saturation fewer VIRTUAL seconds fit the REAL budget, so the climb
truncated (e.g. stalled at CONFIG_13) and the SAME seed produced a DIFFERENT
trajectory idle-vs-hammered. The fix replaces that with a virtual-time bound
(read from the relay vstamp) + completion + a generous real watchdog.

WHAT THIS TEST ASSERTS (contracts that FAIL on the pre-fix harness)
-------------------------------------------------------------------
  [1] read_relay_virtual_seconds() parses the relay vstamp stats line and
      returns virtual seconds = min(a2b_vstamp, b2a_vstamp) / 48000 -- the
      conservative virtual-clock FLOOR. (Pre-fix: this function does not exist.)

  [2] The virtual-second reading is INDEPENDENT of how much real time has
      elapsed: the same relay log yields the same virtual seconds whether read
      at real T+0s or after an arbitrary real delay. (Codifies the property the
      whole fix delivers; pre-fix run loop violated it by construction.)

  [3] The run-bound DECISION fires on VIRTUAL time, not real time: a simulated
      monitor loop that bounds on read_relay_virtual_seconds() stops at the
      VIRTUAL budget regardless of how slow real time advances (host load).
      Two simulated runs with identical virtual-time evolution but DIFFERENT
      real-time evolution (idle vs hammered) terminate at the SAME virtual
      second. (Pre-fix `while time.time()-start < args.secs` terminates at
      DIFFERENT virtual seconds for the two -> trajectory diverges.)

  [4] Completion bound: a byte target reached before the virtual budget ends
      the run early with bounded_by="completion".

  [5] Wedge guard: a frozen virtual clock (relay stopped forwarding) is detected
      by the vclock-stall watchdog, distinct from a clean completion.

Run:  python tools/sim/test_sim_virtual_time_bound.py
Exits 0 on PASS, 1 on FAIL.
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_arq_channel as H

FS = 48000.0
CHUNK = 1024


def _vstamp(chunks):
    """Relay vstamp for a per-direction chunk count (sim_channel_relay.py:980)."""
    return chunks * CHUNK


def _relay_log(a2b_chunks, b2a_chunks):
    """Synthesize a relay log whose LAST a2b/b2a stats lines carry the given
    per-direction chunk counts (the relay emits these every 500 chunks;
    sim_channel_relay.py:998-1002). Returns the file contents."""
    lines = ["[00:00:00] relay listening on 127.0.0.1:52100 SNR3k=12.00dB\n"]
    # emit a few monotonic stats lines per direction, last = the target count
    for c in range(500, a2b_chunks + 1, 500):
        lines.append(
            f"[00:00:01] a2b: {c} chunks ({c*CHUNK/FS:.1f}s) "
            f"vstamp={_vstamp(c)} split=+0 P_sig=0.02250 noise_std=0.001 txpeak=0.15\n")
    for c in range(500, b2a_chunks + 1, 500):
        lines.append(
            f"[00:00:01] b2a: {c} chunks ({c*CHUNK/FS:.1f}s) "
            f"vstamp={_vstamp(c)} split=+0 P_sig=0.02250 noise_std=0.001 txpeak=0.15\n")
    return "".join(lines)


def _write_tmp(contents):
    fd, path = tempfile.mkstemp(suffix=".log", prefix="relay_")
    with os.fdopen(fd, "w") as f:
        f.write(contents)
    return path


def test_1_parse_floor():
    """[1] virtual seconds = min(a2b, b2a) / FS (conservative floor)."""
    # a2b ahead (3000 chunks=64.0s), b2a behind (2500 chunks=53.3s).
    path = _write_tmp(_relay_log(3000, 2500))
    try:
        vsecs, ok = H.read_relay_virtual_seconds(path)
    finally:
        os.remove(path)
    assert ok, "expected ok=True once both directions logged a vstamp"
    expect = _vstamp(2500) / FS          # the FLOOR (slower direction)
    assert abs(vsecs - expect) < 1e-6, f"floor parse wrong: {vsecs} != {expect}"
    # symmetric case
    path = _write_tmp(_relay_log(4000, 4000))
    try:
        vsecs2, ok2 = H.read_relay_virtual_seconds(path)
    finally:
        os.remove(path)
    assert ok2 and abs(vsecs2 - _vstamp(4000) / FS) < 1e-6, "symmetric parse wrong"
    print("  [1] PASS  parse min-floor from relay vstamp")


def test_2_independent_of_real_time():
    """[2] same log -> same virtual seconds regardless of real delay between
    reads (the reading does NOT consult the wall clock)."""
    import time
    path = _write_tmp(_relay_log(3500, 3500))
    try:
        v0, _ = H.read_relay_virtual_seconds(path)
        time.sleep(0.25)                 # arbitrary real-time advance
        v1, _ = H.read_relay_virtual_seconds(path)
    finally:
        os.remove(path)
    assert v0 == v1, f"virtual read drifted with real time: {v0} != {v1}"
    assert abs(v0 - _vstamp(3500) / FS) < 1e-6
    print("  [2] PASS  virtual-second read is real-time-independent")


class _FakeClock:
    """A virtual clock whose value we drive directly, advancing per-tick by a
    rate that we can make idle (1.0) or 'hammered' (slow real time per virtual
    tick) -- exactly the host-load axis. The relay log is regenerated each tick
    from the current virtual chunk count."""
    def __init__(self, vsec_per_tick):
        self.vsec_per_tick = vsec_per_tick
        self.vsec = 0.0

    def tick(self):
        self.vsec += self.vsec_per_tick

    def log_path(self):
        chunks = int(self.vsec * FS / CHUNK)
        # round DOWN to the 500-chunk stats cadence so the last emitted line is
        # realistic; ensure at least one line once past 500 chunks.
        return _write_tmp(_relay_log(chunks, chunks))


def _simulate_run(budget_vsecs, vsec_per_tick, real_sec_per_tick,
                  target_bytes=0, bytes_per_tick=0, freeze_after_vsec=None):
    """Drive the SAME virtual-time-bound decision the harness monitor loop uses,
    in isolation. Returns (stop_vsec, bounded_by). real_sec_per_tick models host
    load: a hammered host advances less virtual time per real second, but here we
    decouple them to PROVE the bound ignores real time. freeze_after_vsec models
    a wedged relay (virtual clock stops)."""
    clk = _FakeClock(vsec_per_tick)
    real = 0.0
    rx = 0
    real_watchdog_s = max(budget_vsecs * H.REAL_WATCHDOG_MULT, H.REAL_WATCHDOG_FLOOR)
    last_v = -1.0
    real_at_last_vadvance = 0.0
    bounded_by = "virtual_secs"
    for _ in range(100000):
        # advance virtual clock unless frozen
        if freeze_after_vsec is None or clk.vsec < freeze_after_vsec:
            clk.tick()
        real += real_sec_per_tick
        rx += bytes_per_tick

        path = clk.log_path()
        try:
            vsecs, ok = H.read_relay_virtual_seconds(path)
        finally:
            os.remove(path)

        # (a) virtual budget
        if ok and vsecs >= budget_vsecs:
            return vsecs, "virtual_secs"
        # (b) completion
        if target_bytes > 0 and rx >= target_bytes:
            return vsecs, "completion"
        # (e) vclock stall (frozen relay)
        if ok and vsecs > last_v + 1e-6:
            last_v = vsecs
            real_at_last_vadvance = real
        elif ok and (real - real_at_last_vadvance) > H.VCLOCK_STALL_REAL_S:
            return vsecs, "vclock_stall"
        # (d) generous real watchdog
        if real > real_watchdog_s:
            return vsecs, "real_watchdog"
    raise AssertionError("simulated run did not terminate")


def test_3_load_independent_bound():
    """[3] THE decisive contract: same virtual evolution, DIFFERENT real
    evolution (idle vs hammered) -> SAME virtual stop-second. The pre-fix
    `while time.time()-start < args.secs` would stop the hammered run at a LOWER
    virtual second (fewer virtual secs fit the real budget)."""
    budget = 120.0
    vstep = 5.0                          # 5 virtual s / tick (same for both)
    # IDLE: real time ~= virtual time (FTRT but 1:1 here for clarity)
    idle_vstop, idle_by = _simulate_run(budget, vstep, real_sec_per_tick=0.05)
    # HAMMERED: real time advances 50x slower per virtual tick (host saturated)
    hammered_vstop, hammered_by = _simulate_run(budget, vstep, real_sec_per_tick=2.5)

    assert idle_by == "virtual_secs", f"idle bounded_by={idle_by}"
    assert hammered_by == "virtual_secs", f"hammered bounded_by={hammered_by}"
    assert idle_vstop == hammered_vstop, (
        f"VIRTUAL stop diverged with host load: idle={idle_vstop} "
        f"hammered={hammered_vstop} -> trajectory would differ (THE BUG)")
    assert idle_vstop >= budget, f"stopped before budget: {idle_vstop}"
    # And the pre-fix real-time bound WOULD diverge: prove the counterfactual.
    # A real-time-bounded loop stops when real >= budget; with the two different
    # real_sec_per_tick the virtual second at that point differs.
    def realtime_bounded_vstop(real_sec_per_tick):
        v = 0.0
        r = 0.0
        while r < budget:
            v += vstep
            r += real_sec_per_tick
        return v
    rt_idle = realtime_bounded_vstop(0.05)
    rt_ham = realtime_bounded_vstop(2.5)
    assert rt_idle != rt_ham, (
        "counterfactual broken: a real-time bound should diverge under load")
    print(f"  [3] PASS  virtual bound load-independent "
          f"(idle==hammered=={idle_vstop}s); real-time bound WOULD diverge "
          f"(idle={rt_idle} vs hammered={rt_ham})")


def test_4_completion_bound():
    """[4] byte target reached before the virtual budget -> bounded_by=completion."""
    budget = 1000.0                      # large so completion wins
    vstop, by = _simulate_run(budget, vsec_per_tick=5.0, real_sec_per_tick=0.05,
                              target_bytes=10000, bytes_per_tick=1000)
    assert by == "completion", f"expected completion, got {by} at v={vstop}"
    assert vstop < budget, "completion should end before the virtual budget"
    print(f"  [4] PASS  completion bound (stopped at v={vstop:.0f}s on byte target)")


def test_5_wedge_guard():
    """[5] frozen virtual clock (wedged relay) -> vclock_stall, not a clean stop."""
    budget = 1000.0
    # freeze virtual time early; real time keeps advancing -> stall watchdog fires
    vstop, by = _simulate_run(budget, vsec_per_tick=5.0, real_sec_per_tick=5.0,
                              freeze_after_vsec=30.0)
    assert by == "vclock_stall", f"expected vclock_stall, got {by} at v={vstop}"
    assert vstop < budget, "wedge should abort before the virtual budget"
    print(f"  [5] PASS  wedge guard (vclock_stall at frozen v={vstop:.0f}s)")


def main():
    print("test_sim_virtual_time_bound.py")
    test_1_parse_floor()
    test_2_independent_of_real_time()
    test_3_load_independent_bound()
    test_4_completion_bound()
    test_5_wedge_guard()
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
