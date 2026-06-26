#!/usr/bin/env python3
"""
ra_cleanup.py - CONCURRENCY-SAFE process cleanup for the real-audio snd-aloop
substrate.

Why this exists
---------------
Every real-audio driver (arq_realaudio.py and friends) and every spawner used to
do an UNCONDITIONAL, GLOBAL:

    pkill -9 -f 'mercury -m ARQ'

at startup. That reaps EVERY mercury on the box, including ones belonging to a
DIFFERENT agent's concurrent A/B cell on a DISJOINT card. Card/substream
isolation does NOT protect against a global pkill - the kill matches by argv
substring, not by audio device. This is the contamination source proven by
diagnostic wf_18b9f890 (a sibling's cohort got SIGKILLed mid-run, producing
0-delivery cells that looked like a modem regression).

The fix
-------
A run/cohort OWNS a specific, disjoint set of ALSA cables (card + substreams)
and TCP ports. snd-aloop wires playback(dev0,subN) <-> capture(dev1,subN), so a
run that owns substreams {S0..S3} on card C uses exactly the device strings
  hw:C,0,S  and  hw:C,1,S   for S in {S0..S3}
on its two mercury -x alsa processes AND its bridge. Those device strings appear
VERBATIM in the target's /proc/<pid>/cmdline. A run also owns its rsp/cmd TCP
ports (and port+1 data sockets), which appear as "-p <port>" in mercury argv.

scoped_cleanup() scans /proc and SIGKILLs only processes whose cmdline contains
one of THIS owner's device strings or "-p <port>" tokens. A sibling on a disjoint
card/subs/port set can never match, so it is never reaped. The caller's own
process tree (self + descendants) is excluded as a belt-and-braces guard.

This is a drop-in replacement for the old global pkill: it clears THIS owner's
own stale leftovers (e.g. a crashed previous run that still holds the cables)
without touching anyone else.
"""
import os
import signal
import time


def _own_pids():
    """PIDs to never kill: this process, its parent, and its descendants."""
    me = os.getpid()
    keep = {me, os.getppid()}
    # include descendants (defensive; normally none yet at cleanup time)
    try:
        children = {}
        for pid in _iter_pids():
            ppid = _ppid_of(pid)
            if ppid is not None:
                children.setdefault(ppid, []).append(pid)
        stack = [me]
        while stack:
            p = stack.pop()
            for c in children.get(p, []):
                if c not in keep:
                    keep.add(c)
                    stack.append(c)
    except OSError:
        pass
    return keep


def _iter_pids():
    for name in os.listdir("/proc"):
        if name.isdigit():
            yield int(name)


def _ppid_of(pid):
    try:
        with open(f"/proc/{pid}/stat", "rb") as f:
            data = f.read()
        # stat: pid (comm) state ppid ...   comm may contain spaces/parens,
        # so split after the last ')'.
        rparen = data.rfind(b")")
        fields = data[rparen + 2:].split()
        return int(fields[1])  # ppid is field index 1 after state
    except (OSError, ValueError, IndexError):
        return None


def _cmdline(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return f.read().replace(b"\x00", b" ").decode("utf-8", "replace")
    except OSError:
        return ""


def own_tokens(card, subs, ports):
    """Build the set of argv substrings that UNIQUELY identify processes owned by
    a run on this card / substreams / ports.

    card  : snd-aloop card name (e.g. "Loopback" or "Loopback_3") or index str.
    subs  : iterable of the 4 substream indices this run owns.
    ports : iterable of TCP ports this run owns (rsp_port, cmd_port).

    Returns a list of substrings; a process is OURS iff its cmdline contains at
    least one of these AND it is a mercury or realaudio_bridge process.
    """
    toks = []
    for s in subs:
        # both playback (dev 0) and capture (dev 1) device strings for this cable
        toks.append(f"hw:{card},0,{s}")
        toks.append(f"hw:{card},1,{s}")
    for p in ports:
        # mercury control port and its data port (port+1)
        toks.append(f"-p {p}")
        toks.append(f"-p {p + 1}")
    return toks


def scoped_cleanup(card, subs, ports, settle=1.0, verbose=False):
    """SIGKILL only the mercury / realaudio_bridge processes that OWN one of this
    run's ALSA cables or TCP ports. Never touches a sibling on disjoint cables.

    Returns the list of (pid, cmdline) actually killed (for logging / proof).
    """
    toks = own_tokens(card, subs, ports)
    keep = _own_pids()
    killed = []
    for pid in list(_iter_pids()):
        if pid in keep:
            continue
        cl = _cmdline(pid)
        if not cl:
            continue
        is_target = ("mercury -m ARQ" in cl) or ("realaudio_bridge_s32.py" in cl)
        if not is_target:
            continue
        if any(tok in cl for tok in toks):
            try:
                os.kill(pid, signal.SIGKILL)
                killed.append((pid, cl))
                if verbose:
                    print(f"[ra_cleanup] killed {pid}: {cl[:120]}")
            except OSError:
                pass
    if killed:
        time.sleep(settle)
    return killed


def scoped_cleanup_cells(cells, settle=1.0, verbose=False):
    """Cohort-level cleanup for a spawner: clear stale leftovers for EVERY cell
    THIS spawner is about to launch (its own plan), and nothing else.

    `cells` is an iterable of dicts each with keys: card, subs, rsp_port,
    cmd_port (the same per-cell plan dicts the spawners already build).
    """
    all_killed = []
    for c in cells:
        ports = [c["rsp_port"], c["cmd_port"]]
        all_killed += scoped_cleanup(c["card"], c["subs"], ports,
                                     settle=0.0, verbose=verbose)
    if all_killed:
        time.sleep(settle)
    return all_killed
