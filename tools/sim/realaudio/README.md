# Faithful real-audio substrate (snd-aloop)

The **most faithful** Mercury channel simulator available off-RF: two stock
`-x alsa` Mercury processes (a real commander + a real responder) talk to each
other through an **IONOS-over-real-audio bridge** on Linux `snd-aloop`. The ONLY
thing between the two modems is `Channel.process()` — the same channel DSP the
`-x sim` TCP relay uses (`sim_channel_relay.py`), but carried over a real ALSA
audio path at Mercury's actual `-x alsa` wire format (INT32 / 48000 Hz / 2ch,
raw `hw:` — no `plug` resample/requantize).

Proven in `wf_1e5c12f1`: a real modem round-trip CONNECTS + DELIVERS over it.

## Why this substrate matters: it is LOAD-IMMUNE

Each bridge capture read is **kernel-clocked by the snd-aloop hardware timer**,
not by the contended CPU. Pinned ROBUST_0 idle vs `stress-ng --cpu 24` gave
**bit-identical delivered frames+bytes**. Consequences:

- **Concurrency is free and safe.** N independent runs (each its own 4 cables,
  2 mercury processes, 1 bridge, disjoint TCP ports) overlap with total
  wall-clock ≈ ONE run's duration, and each run's delivered bytes match a
  single-run reference. An A/B of 12×2 cells finishes in ~5 min, not ~74.
- This **retires** the old virtual-clock "one sim per box" starvation rule —
  that limitation was specific to the decode-consumption-driven virtual clock
  of the TCP relay, NOT this real-audio path.
- Parallelize by default. A 36-commit linear hunt is ~6 min concurrent, not ~24
  serial.

## Files (this directory)

| File | Role |
|------|------|
| `realaudio_bridge_s32.py` | The IONOS bridge. Opens raw `hw:` S32_LE/2ch/48k, recovers the float64 passband (`/INT_MAX`), runs `Channel.process()` verbatim, re-scales (`*INT_MAX`), writes back. Imports `Channel/PROFILES/WGN_TO_SNR3K` from the parent `tools/sim/sim_channel_relay.py`. |
| `arq_realaudio.py` | One A/B cell driver: launches the bridge + 2 `-x alsa` mercury processes on one run's 4 cables, drives the canonical ctrl/data TCP protocol (MYCALL/LISTEN/CONNECT), measures connect + delivered bytes/frames. Mirrors `tools/sim_arq_channel.py` exactly; the ONLY change is the audio transport. |
| `parallel_spawner.py` | Launches N concurrent `arq_realaudio.py` cells, allocates disjoint cards/substreams/ports, summarizes connect+delivery. This is the embarrassingly-parallel substrate entry point. |
| `ra_cleanup.py` | **Concurrency-safe scoped process cleanup. READ THE PKILL FIX SECTION BELOW.** |

`sim_channel_relay.py` (the channel DSP) lives one level up in `tools/sim/` and
is imported by the bridge — it is shared with the `-x sim` TCP path, not copied.

## snd-aloop card / cable / port layout

`snd-aloop` internally wires **playback(dev 0, sub N) ⇄ capture(dev 1, sub N)**.
So one "cable" = one substream index used on both dev 0 (play) and dev 1 (cap).
**One run = 4 cables** (FWD + REV, each needs a cap + a play substream):

```
Commander  -o hw:<card>,0,<S0> (TX)   -i hw:<card>,1,<S3> (RX)
Responder  -i hw:<card>,1,<S1> (RX)   -o hw:<card>,0,<S2> (TX)
Bridge FWD: cap hw:<card>,1,<S0> -> Channel.process -> play hw:<card>,0,<S1>  (CMD->RSP)
Bridge REV: cap hw:<card>,1,<S2> -> Channel.process -> play hw:<card>,0,<S3>  (RSP->CMD)
```

A card supports up to **8 substreams**, so **2 runs fit per card**. N runs need
`ceil(N/2)` cards. Card k carries runs 2k (subs 0,1,2,3) and 2k+1 (subs 4,5,6,7).

Provision snd-aloop (run on the fleet box, needs root):

```
# enough cards for N runs (ceil(N/2) cards x 8 substreams). Or:
python3 parallel_spawner.py --n <N> --print-setup
# prints, e.g.:
sudo modprobe -r snd-aloop; sudo modprobe snd-aloop index=0,1,...,k enable=1,1,... pcm_substreams=8,8,...
cat /proc/asound/cards   # verify
```

snd-aloop names successive cards `Loopback`, `Loopback_1`, `Loopback_2`, …

**TCP ports** (each run disjoint): run i → `rsp = BASE + 10*i`, `cmd = BASE + 10*i + 4`
(data sockets are port+1; +10 spacing leaves headroom). Default BASE = 7100.

## How to run an A/B

Single cell (one run, one channel point):

```
python3 arq_realaudio.py --bin /path/to/mercury \
    --card Loopback --subs 0,1,2,3 --rsp-port 7002 --cmd-port 7006 \
    --start-cfg 100 --secs 130 --payload 512 \
    --cell WGN:35 --profile wgn        # or --passthrough for a bit-exact cable
```

Parallel cohort (the normal mode — N cells at once, ~one cell's wall-clock):

```
# arm A (legacy) and arm B (a fix) are just different --bin / --env per cell.
python3 parallel_spawner.py --n 12 --bin /path/to/mercury \
    --secs 130 --payload 512 --start-cfg 100 --no-gearshift \
    --cell WGN:55 --profile wgn \
    --logdir /tmp/raspeed/logs --out /tmp/raspeed/PARALLEL_RESULT.json
```

For an A/B the convention is one spawner/cohort per arm (each arm its own
`--bin` and/or `--env KEY=VAL` mercury env injection — `arq_realaudio.py --env`
carries e.g. `MERCURY_INBAND_RATE=...` for a redesign arm, legacy leaves it
unset), on disjoint cards/ports so both arms can run truly concurrently.
`--cell WGN:<dB>` sets the SNR (the bridge adds `WGN_TO_SNR3K`); `--profile`
selects the fade profile from `sim_channel_relay.py PROFILES`; `--passthrough`
disables the channel entirely for a P0 bit-exact reference. Results are JSON:
per-run `connected`, `rx_bytes`, `rsp_nreceived_frames`, `configs_seen`.

The substrate is fleet-local: the boxes clone Mercury to local disk
(`~/raspeed/mercury`, `~/mercury`) — the `--bin` / `--logdir` / `--out` defaults
reflect that and are overridable. Run it on a fleet R730 (snd-aloop + ALSA),
not on Windows.

## ★ THE PKILL FIX — never global-pkill (`ra_cleanup.py`)

**This is the load-bearing reliability fix; do not regress it.**

### What was wrong

Every driver and spawner used to start with an **unconditional, global**:

```
pkill -9 -f 'mercury -m ARQ'
```

That reaps **EVERY** mercury on the box — including a **different agent's**
concurrent A/B cell on a **disjoint** card. Card/substream/port isolation does
**NOT** protect against a global pkill: the kill matches by argv substring, not
by audio device. Diagnostic `wf_18b9f890` proved this is the contamination
source — a sibling's cohort got SIGKILLed mid-run, producing 0-delivery cells
that **looked like a modem regression** but were a harness fratricide. Because
this substrate's whole value is running many concurrent runs, a global pkill is
directly self-defeating.

### The fix: scoped, per-card/substream/port cleanup

A run/cohort **OWNS** a specific disjoint set of ALSA cables (card + 4
substreams) and TCP ports. Those device strings (`hw:<card>,0,<S>`,
`hw:<card>,1,<S>`) and port tokens (`-p <port>`) appear **verbatim** in the
target's `/proc/<pid>/cmdline`. `ra_cleanup.py` scans `/proc` and SIGKILLs
**only** mercury / `realaudio_bridge_s32.py` processes whose cmdline contains one
of **THIS owner's** device strings or `-p <port>` tokens. A sibling on a disjoint
card/subs/port set can never match, so it is **never** reaped. The caller's own
process tree (self + parent + descendants) is excluded as a belt-and-braces
guard. It still clears THIS owner's own stale leftovers (e.g. a crashed previous
run still holding the cables) — a true drop-in replacement for the old global
pkill.

```python
from ra_cleanup import scoped_cleanup        # one run:    card, subs, [rsp,cmd] ports
from ra_cleanup import scoped_cleanup_cells  # cohort:     a list of per-cell plan dicts
scoped_cleanup(args.card, subs, [rsp_port, cmd_port], settle=1.5)
scoped_cleanup_cells(plan, settle=2.0)
```

`arq_realaudio.py` calls `scoped_cleanup` (unless `--no-kill`); the spawner runs
each child with `--no-kill` and does ONE `scoped_cleanup_cells` for its own plan
before launch. **RULE: never global-pkill in this substrate. Any new driver or
spawner MUST use `ra_cleanup.py` and scope the kill to the cells it owns.**

## Provenance

Authored on the fleet at `/home/kameron/raspeed/realaudio/`; canonical
post-pkill-fix versions wired into the repo here. The bridge/relay channel math
is shared with the `-x sim` TCP path (`tools/sim/sim_channel_relay.py`).
