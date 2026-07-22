#!/usr/bin/env python3
"""SACK lossy-channel A/B sweep — does --enable-sack WIN on a lossy channel?
=============================================================================
Compares `--enable-sack` vs no-SACK, apples-to-apples, on the IONOS Pi
testbed via the butler (localhost:7700). Fixed config, NO gearshift — the
same recipe phase0_baseline.py uses for the PI track (`-s <cfg> -Q 0 -M auto
-F off --skip-turbo-reverse`). The ONLY variable between the A and B cell of
a pair is the presence of `--enable-sack` on both Mercury instances.

SACK's whole purpose is partial-batch recovery: when a batch loses frames,
no-SACK pays a full-batch-retransmit penalty per loss, while SACK retransmits
only the missing frames. This sweep measures whether that theoretical win is
real and where the clean<->lossy crossover sits.

Design (validation methodology):
  * Fixed config (default WB_CFG15), no gearshift — apples-to-apples.
  * Channel sweep: a clean baseline + progressively lossier points. Each
    point is a list of documented-valid IONOS serial commands (WGN:N,
    MPG/MPM/MPP/MPD, FADE DEPTH/FREQ). CH:FM / CH:THRU are INVALID and
    never used.
  * INTERLEAVED A/B: for each channel point we alternate sack / nosack runs
    (sack, nosack, sack, nosack, ...) so slow channel drift biases neither
    mode. Runs are NOT batched mode-first.
  * >=3 runs per (channel, mode) cell (--runs, upgrade to 5 if variance high).
  * MECHANISM capture: every run downloads the full CMD + RSP logs and we
    parse throughput, per-batch frame-loss, retransmit counts (nReSent_data),
    [CMD-SACK]/[CMD-RETX] events, [RX-SACK] ldpc= events, batch cycle times.
  * Stale-log trap avoided: logs are pulled per-run with a deterministic
    per-run filename, downloaded into a space-free temp dir then moved
    (butler DOWNLOAD splits args on whitespace). Matches timing_pull_pi_logs.py
    and mercury_ionos_sack_test.py.

Usage:
  python tools/sack_lossy_ab.py --out <json> [--config WB_CFG15]
      [--runs 3] [--duration 90] [--points clean,wgn18,wgn15,mpm15]
      [--settle-s 30]

bps is measured over (duration - settle_s) starting from settle_s,
NOT over the full duration. The sackv2 Axis-2 ring-clean ramp from
batch=25 needs 8 good batches per up-step; at CFG15 (~12 s/batch) the
first 30-90 s of any window is settling, not steady-state. Skipping
the first --settle-s seconds (default 30) collapses the artifactual
variance that dominated short-window σ on n=2 sackv2 cells.

All hardware access goes through the butler. No direct SSH / paramiko / serial.
"""
import argparse, hashlib, json, os, re, shutil, socket, tempfile, threading, time, sys

if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Migrated into the mercury repo at mercury/tools/sim/ (was tools/ in the outer
# workspace). Anchor paths to the mercury repo root and the outer workspace root
# explicitly so the default payload lookup (pg84.txt) keeps resolving to the real
# file regardless of this script's depth.
MERCURY_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))   # mercury/tools/sim -> mercury
WORKSPACE    = os.path.dirname(MERCURY_ROOT)                  # mercury -> workspace root

# ── Testbed wiring (same as phase0_baseline.py) ───────────────────────────────
BUTLER       = ('localhost', 7700)
PI_RSP_HOST  = '192.168.2.217'   # rpi1 = Responder
PI_CMD_HOST  = '192.168.2.215'   # rpi2 = Commander
PI_RSP_PORT  = 8400
PI_CMD_PORT  = 8300
PI_MERCURY   = '~/mercury-dev/mercury'
PI_AUDIO_DEV = 'plughw:Audio'    # Fe-Pi card by NAME (index not stable)

CMD_LOG = '/tmp/m_cmd.log'
RSP_LOG = '/tmp/m_rsp.log'

# Config table: label -> (config_id, is_nb)
CONFIGS = {
    # CONFIG_0: WB BPSK rate-1/16 (the deepest-SNR OFDM data config, the gate
    # fix's primary target). Pinned for the SKIP-VAR gate-fix acquire-vs-decode
    # sweep (COMBINED HW validation 2026-06-02). Not narrowband.
    'WB_CFG0':  (0,  False),
    # CONFIG_2: WB QPSK-class rate-low, second-deepest. Added for the OFDM
    # coherent-acquisition deep-cell A/B (WIN CAMPAIGN inc.2, 2026-06-02) so
    # the "+CONFIG_2 if it acquires" arm is available. Not narrowband.
    'WB_CFG2':  (2,  False),
    'WB_CFG4':  (4,  False),
    'WB_CFG10': (10, False),
    'WB_CFG14': (14, False),
    'WB_CFG15': (15, False),
    'WB_CFG16': (16, False),
    'WB_CFG17': (17, False),
    # ROBUST_0 (MFSK + rate-1/16 LDPC, config id 100). Pinning the robust
    # tier lets the SACK harness drive the weak-signal floor directly, the
    # same way the calibrator pins configs. Used by the bessel-I0 / mini-
    # Moose floor A/Bs (2026-05-31).
    'WB_ROBUST0': (100, False),
    # ROBUST_2 (MFSK M16x2 + rate-1/4 LDPC, config id 102). Same robust-mode
    # requirement as the ULTRA rungs below: an explicit -s does NOT auto-enable
    # robust (main.cc:2239 gates auto-enable on !explicit_config), so the caller
    # MUST pass MERCURY_EXTRA_FLAGS="-R". PINNED (no gearshift). Added 2026-06-02
    # for the M16x2 stream-energy-combiner data-frame ACQUISITION HW validation
    # (the combiner only acts on nStreams>=2; ROBUST_2 is the M16x2 geometry).
    'WB_ROBUST2': (102, False),
    # ROBUST_RA (cfg103): the WIN-CAMPAIGN integrated "-10 data mode" — 16-MFSK x2
    # frequency-diversity geometry + GF(16)-RA R1/4 Q-ary data FEC (NOT binary LDPC).
    # OFF-LADDER, pin-only: like ROBUST_2/ULTRA it does NOT auto-enable robust on an
    # explicit -s (main.cc GUI gate 2236-2332 needs robust_mode_enabled||-R), so the
    # caller MUST pass MERCURY_EXTRA_FLAGS="-R" (and --robust-batch N for the incr2
    # chokepoint-lift dwell). PINNED (no gearshift). repfact is fixed at 3 (=R1/4)
    # in common_defines.h ROBUST_RA_REPFACT. Used by the WIN-CAMPAIGN -10 REACH HW
    # validation (integrated incr1+incr2, branch win/integ-incr1-incr2 @fd5c698).
    'WB_ROBUST_RA': (103, False),
    # ULTRA baud-scaled MFSK rungs (sim/ultra-baud-rungs @44701d6, config IDs
    # 200-203, ULTRA_0=deepest=K=8 ... ULTRA_3=shallowest=K=1). Reuse the
    # ROBUST_0-class MFSK DATA PHY with depth from per-config Nfft (baud-scaling),
    # so they REQUIRE robust mode: pass MERCURY_EXTRA_FLAGS="-R" (an explicit -s
    # does NOT auto-enable robust — main.cc:2239 gates auto-enable on
    # !explicit_config). PINNED (no gearshift) so the deadlock + climb-out-of-
    # ROBUST_0 bugs do not apply. Used by the DEEP-ULTRA HW validation 2026-06-01.
    'WB_ULTRA0': (200, False),   # K=8  Nfft=2048  ~-21 dB SNR3k (deepest)
    'WB_ULTRA1': (201, False),   # K=4  Nfft=1024  ~-19/-20 dB (workhorse)
    'WB_ULTRA2': (202, False),   # K=2  Nfft=512   ~-16 dB
    'WB_ULTRA3': (203, False),   # K=1  Nfft=256   ~-13 dB (bridges ROBUST_0)
}

# ── Channel points ────────────────────────────────────────────────────────────
# Each point = ordered list of documented-valid IONOS serial commands.
# CH:FM / CH:THRU are INVALID (memory's ionos_commands.md) and never appear.
# We always pin gains (IN:1 OUT:1) and reset fade/offset so a point fully
# defines the channel regardless of what ran before it.
_BASE_GAINS = ['CH1 IN:1', 'CH2 IN:1', 'CH1 OUT:1', 'CH2 OUT:1', 'BANDWIDTH:3000']
_FLAT       = ['FADE DEPTH:0', 'FADE FREQ:0', 'OFFSET:0']

CHANNEL_POINTS = {
    # Clean baseline — matches phase0 'clean'. Expect ~0% per-batch loss.
    'clean':  ['WGN:40'] + _FLAT + _BASE_GAINS,
    # Progressively lossier AWGN points. WB_CFG15 is rate-0.875 32QAM-ish;
    # lowering SNR drives real per-frame LDPC failures without dropping the
    # control link (turboshift control frames are far more robust).
    # CALIBRATED 2026-05-14: WGN:40 ~= 0% loss, WGN:20 = 100% data loss
    # (full data-frame failure, link still up). The WB_CFG15 cliff is in
    # the WGN:24-38 band — these points sweep it.
    'wgn36':  ['WGN:36'] + _FLAT + _BASE_GAINS,
    'wgn32':  ['WGN:32'] + _FLAT + _BASE_GAINS,
    'wgn30':  ['WGN:30'] + _FLAT + _BASE_GAINS,
    'wgn28':  ['WGN:28'] + _FLAT + _BASE_GAINS,
    'wgn26':  ['WGN:26'] + _FLAT + _BASE_GAINS,
    'wgn24':  ['WGN:24'] + _FLAT + _BASE_GAINS,
    'wgn22':  ['WGN:22'] + _FLAT + _BASE_GAINS,
    'wgn20':  ['WGN:20'] + _FLAT + _BASE_GAINS,
    'wgn18':  ['WGN:18'] + _FLAT + _BASE_GAINS,
    'wgn16':  ['WGN:16'] + _FLAT + _BASE_GAINS,
    'wgn14':  ['WGN:14'] + _FLAT + _BASE_GAINS,
    'wgn12':  ['WGN:12'] + _FLAT + _BASE_GAINS,
    'wgn11':  ['WGN:11'] + _FLAT + _BASE_GAINS,   # RF-ground CONFIG_0 decode-floor extension (2026-06-02)
    'wgn10':  ['WGN:10'] + _FLAT + _BASE_GAINS,
    'wgn9':   ['WGN:9']  + _FLAT + _BASE_GAINS,   # odd points: GATE-fix CONFIG_0 cliff sweep (2026-06-02)
    'wgn8':   ['WGN:8']  + _FLAT + _BASE_GAINS,
    'wgn7':   ['WGN:7']  + _FLAT + _BASE_GAINS,
    'wgn6':   ['WGN:6']  + _FLAT + _BASE_GAINS,
    'wgn5':   ['WGN:5']  + _FLAT + _BASE_GAINS,
    'wgn4':   ['WGN:4']  + _FLAT + _BASE_GAINS,
    'wgn2':   ['WGN:2']  + _FLAT + _BASE_GAINS,
    'wgn0':   ['WGN:0']  + _FLAT + _BASE_GAINS,
    'wgn-2':  ['WGN:-2'] + _FLAT + _BASE_GAINS,
    'wgn-4':  ['WGN:-4'] + _FLAT + _BASE_GAINS,
    'wgn-6':  ['WGN:-6'] + _FLAT + _BASE_GAINS,
    'wgn-8':  ['WGN:-8'] + _FLAT + _BASE_GAINS,
    'wgn-10': ['WGN:-10'] + _FLAT + _BASE_GAINS,
    # Deeper sub-cliff cells (added 2026-05-31 for the bessel-I0 floor A/B).
    # The cliff sits between WGN:-8 (working) and WGN:-10 (~5 B/min). These
    # exercise the MARGINAL-frame regime (LDPC iter>0) where the demap-metric
    # quality changes the decode outcome.
    'wgn-12': ['WGN:-12'] + _FLAT + _BASE_GAINS,
    'wgn-14': ['WGN:-14'] + _FLAT + _BASE_GAINS,
    # Even deeper cells (added 2026-06-01 for the suffix-FEC PRODUCTION
    # establishment re-validate). The FORCE-ON run proved the enhanced
    # CONNECT establishes to WGN:-16; -18 probes the floor below it.
    'wgn-16': ['WGN:-16'] + _FLAT + _BASE_GAINS,
    'wgn-18': ['WGN:-18'] + _FLAT + _BASE_GAINS,
    # Deepest cells (added 2026-06-01 for the DEEP-ULTRA HW validation). The
    # ULTRA_0 (K=8) data+establishment cliff is predicted ~-21 dB SNR3k in sim
    # (per-config-nfft-ultra-rungs.md §5.3/§6); these probe the #2-doorbell floor.
    'wgn-19': ['WGN:-19'] + _FLAT + _BASE_GAINS,
    'wgn-20': ['WGN:-20'] + _FLAT + _BASE_GAINS,
    'wgn-21': ['WGN:-21'] + _FLAT + _BASE_GAINS,
    'wgn-22': ['WGN:-22'] + _FLAT + _BASE_GAINS,
    # Multipath points — frequency-selective loss, the realistic HF case
    # SACK was designed for. MPG = 0.1 Hz / 0.5 ms; MPM = 0.5 Hz / 1 ms;
    # MPP = 1 Hz / 2 ms; MPD = 2 Hz / 4 ms. The :N suffix is the SNR in dB —
    # the multipath PROFILE (Doppler/delay) is fixed per mode, N sets the
    # noise floor on top of it.
    'mpg30':  ['MPG:30'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpg26':  ['MPG:26'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpm30':  ['MPM:30'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpm26':  ['MPM:26'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    # 2026-05-14 Track B re-calibration: MPG:26/MPM:26 produced ZERO
    # partial-batch loss for WB_CFG10 (measured SNR ~13.5 dB, all OFDM-OK,
    # 0 SACK events / 18 runs). To reach the genuine 10-40% partial-batch
    # regime the multipath must be DEEPER (MPP 1Hz/2ms, MPD 2Hz/4ms — the
    # 2-4 ms delay spread is comparable to / exceeds the OFDM guard interval
    # so it causes real frequency-selective ISI) AND the SNR lower. These
    # points sweep that. Calibrated below in §2.4.
    'mpp22':  ['MPP:22'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpp18':  ['MPP:18'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpp16':  ['MPP:16'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpp14':  ['MPP:14'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpp12':  ['MPP:12'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpd20':  ['MPD:20'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpd16':  ['MPD:16'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpd14':  ['MPD:14'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpd12':  ['MPD:12'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpm18':  ['MPM:18'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpm16':  ['MPM:16'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    # WIN-CAMPAIGN cfg103 (GF16-RA) REACH-under-FADING validation, 2026-06-03.
    # The 3 standard CCIR Watterson profiles at the noise level that maps to a
    # TRUE -10 dB SNR3k. Per [[testbed-wgn-snr3k-mapping]] the IONOS noise dial
    # reads ~2.4 dB optimistic vs measured SNR3k (channel SNR3k = label + 2.4),
    # so the dial label that yields true -10 dB is -12 (= the WGN:-12 anchor's
    # noise floor) layered on each multipath profile. This is the STRICT
    # comparison: VARA's worksheet MPG/MPM/MPP "-10" row used the IONOS DIAL -10
    # (= true -7.6 dB), so MPx:-12 tests Mercury ~2.4 dB HARDER than VARA's row.
    'mpg-12': ['MPG:-12'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpm-12': ['MPM:-12'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpp-12': ['MPP:-12'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    # Dial-matched to VARA's worksheet "-10" S:N row (IONOS dial -10 = true
    # -7.6 dB SNR3k) for the apples-to-apples-BY-DIAL VARA comparison.
    'mpg-10': ['MPG:-10'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpm-10': ['MPM:-10'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
    'mpp-10': ['MPP:-10'] + ['FADE DEPTH:0', 'FADE FREQ:0'] + _BASE_GAINS,
}


# ── Butler client ─────────────────────────────────────────────────────────────
def b_send(s, cmd, t=40):
    s.sendall((cmd + '\n').encode())
    s.settimeout(t)
    buf = b''
    while True:
        try:
            d = s.recv(65536)
            if not d:
                break
            buf += d
            if b'\n' in buf:
                break
        except socket.timeout:
            break
    return buf.decode('utf-8', 'replace').strip()


def b_lock(label, secs=600):
    bs = socket.socket()
    bs.connect(BUTLER)
    r = b_send(bs, f'LOCK {label} {secs}')
    if not r.startswith('OK'):
        bs.close()
        raise RuntimeError(f'butler LOCK failed: {r}')
    return bs, r.split()[1]


def graceful_stop_mercury(bs, lid, rpi, grace_s=12):
    """Stop Mercury through the Butler without risking an ALSA hard-kill wedge.

    A nonzero SSH result means a process survived the grace interval.  Callers
    must fail the cell/session instead of escalating to SIGKILL.
    """
    grace_s = max(10, int(grace_s))
    cmd = (
        'killall -TERM mercury 2>/dev/null || true; '
        'i=0; '
        f'while pgrep -x mercury >/dev/null && [ "$i" -lt {grace_s} ]; do '
        'sleep 1; i=$((i+1)); done; '
        'if pgrep -x mercury >/dev/null; then '
        'echo MERCURY_SURVIVORS; exit 3; '
        'else echo MERCURY_CLEAN; fi'
    )
    reply = b_send(bs, f'SSH {lid} {rpi} {cmd}', t=grace_s + 15)
    return reply.startswith('OK rc=0'), reply


# ── Mercury TCP helpers ───────────────────────────────────────────────────────
def tcp_connect_retry(host, port, retries=20, delay=2):
    last = None
    for _ in range(retries):
        try:
            s = socket.socket()
            s.settimeout(5)
            s.connect((host, port))
            return s
        except Exception as e:
            last = e
            time.sleep(delay)
    raise ConnectionError(f'{host}:{port} ({last})')


def tcp_send_line(s, msg):
    s.sendall((msg + '\r').encode())


def tcp_recv_drain(s, timeout=2):
    s.settimeout(timeout)
    try:
        return s.recv(4096).decode('utf-8', 'replace')
    except socket.timeout:
        return ''


def tcp_recv_until(s, target, timeout=120):
    # FIN-on-empty defensive: at low SNR mercury sometimes cycles its control
    # socket mid-CONNECT; recv() then returns b'' (FIN). The previous loop
    # silently ignored empty reads but kept spinning, burning CPU until the
    # deadline. Now we sleep briefly on each empty read so the caller still
    # gets the full timeout window for any reconnect/retransmit to land,
    # without a tight spin. If the peer is really gone the deadline bounds
    # the wait.
    s.settimeout(1)
    buf = ''
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            data = s.recv(4096)
            if data:
                buf += data.decode('utf-8', 'replace')
                if target in buf:
                    return buf
            else:
                # Empty read = FIN. Don't bail; pause and re-poll until deadline.
                time.sleep(0.5)
        except socket.timeout:
            continue
        except Exception:
            break
    return buf


def receiver_thread(host, port, result, stop_event, payload=None,
                    connect_timeout=15):
    try:
        s = socket.socket()
        s.settimeout(connect_timeout)
        s.connect((host, port))
        s.settimeout(1)
        total = 0
        mismatches = 0
        first_mismatch = None
        received_hash = hashlib.sha256()
        expected_hash = hashlib.sha256()
        while not stop_event.is_set():
            try:
                data = s.recv(8192)
                if data:
                    received_hash.update(data)
                    if payload:
                        expected = bytes(payload[(total + i) % len(payload)]
                                         for i in range(len(data)))
                        expected_hash.update(expected)
                        chunk_mismatches = sum(a != b for a, b in zip(data, expected))
                        if chunk_mismatches and first_mismatch is None:
                            first_mismatch = total + next(
                                i for i, (a, b) in enumerate(zip(data, expected))
                                if a != b)
                        mismatches += chunk_mismatches
                    total += len(data)
                    result['bytes'] = total
                    result['mismatch_bytes'] = mismatches
                    result['first_mismatch_offset'] = first_mismatch
                else:
                    break
            except socket.timeout:
                continue
            except Exception:
                break
        try:
            s.close()
        except Exception:
            pass
    except Exception as e:
        result['error'] = str(e)
    result['bytes'] = result.get('bytes', 0)
    result['mismatch_bytes'] = result.get('mismatch_bytes', 0)
    result['first_mismatch_offset'] = result.get('first_mismatch_offset')
    if 'received_hash' in locals():
        result['received_sha256'] = received_hash.hexdigest()
        result['expected_sha256'] = expected_hash.hexdigest() if payload else None
    result['byte_exact'] = bool(payload and result['bytes'] > 0 and
                                result['mismatch_bytes'] == 0)


def sender_thread(host, port, payload, stop_event, result, connect_timeout=15):
    try:
        s = socket.socket()
        s.settimeout(connect_timeout)
        s.connect((host, port))
        s.settimeout(5)
        sent = 0
        idx = 0
        while not stop_event.is_set():
            chunk = payload[idx:idx + 1024]
            if not chunk:
                idx = 0
                continue
            try:
                s.sendall(chunk)
                sent += len(chunk)
                idx += len(chunk)
                if idx >= len(payload):
                    idx = 0
            except socket.timeout:
                continue
            except Exception:
                break
        result['tx_bytes'] = sent
        try:
            s.close()
        except Exception:
            pass
    except Exception as e:
        result['tx_error'] = str(e)


# ── Log parsing — the MECHANISM evidence ──────────────────────────────────────
def parse_mercury_logs(cmd_text, rsp_text):
    """Extract the SACK mechanism metrics from a CMD+RSP log pair.

    Returns a dict. Numbers come from Mercury's own counters — cited in the
    fact doc by the log file each value was parsed from.
    """
    m = {}

    # --- cumulative stats (periodic dump, arq_common.cc:5720-5726) ------------
    # We take the LAST occurrence of each = final cumulative counter value.
    def last_int(text, key):
        val = None
        for line in text.split('\n'):
            mo = re.search(re.escape(key) + r'\s*=\s*(-?\d+)', line)
            if mo:
                val = int(mo.group(1))
        return val

    for src, text in (('cmd', cmd_text), ('rsp', rsp_text)):
        m[f'{src}_nSent_data']     = last_int(text, 'stats.nSent_data')
        m[f'{src}_nReSent_data']   = last_int(text, 'stats.nReSent_data')
        m[f'{src}_nAcked_data']    = last_int(text, 'stats.nAcked_data')
        m[f'{src}_nReceived_data'] = last_int(text, 'stats.nReceived_data')
        m[f'{src}_nLost_data']     = last_int(text, 'stats.nLost_data')
        m[f'{src}_nBatches']       = last_int(text, 'stats.nBatches_sent')

    # --- [CMD-SACK] N/M received, K queued for retransmit --------------------
    #   arq_commander.cc:1479. One per partial-batch SACK reception.
    cmd_sack = []
    for mo in re.finditer(r'\[CMD-SACK\]\s+(\d+)/(\d+)\s+received,\s+(\d+)\s+queued',
                          cmd_text):
        rx, tot, retx = int(mo.group(1)), int(mo.group(2)), int(mo.group(3))
        cmd_sack.append({'rx': rx, 'batch': tot, 'queued': retx})
    m['cmd_sack_events'] = cmd_sack
    m['n_cmd_sack_events'] = len(cmd_sack)
    if cmd_sack:
        # per-batch frame loss as observed by the SACK bitmap
        losses = [(e['batch'] - e['rx']) / e['batch']
                  for e in cmd_sack if e['batch'] > 0]
        m['sack_per_batch_loss_mean'] = round(sum(losses) / len(losses), 4) if losses else None
        m['sack_per_batch_loss_max']  = round(max(losses), 4) if losses else None
        m['total_frames_queued_retx'] = sum(e['queued'] for e in cmd_sack)

    # --- [CMD-RETX] Sending N retransmit frames ------------------------------
    #   arq_commander.cc:732. SACK path: count of selective-retransmit frames.
    retx_sent = [int(x) for x in re.findall(r'\[CMD-RETX\]\s+Sending\s+(\d+)\s+retransmit',
                                            cmd_text)]
    m['cmd_retx_bursts'] = len(retx_sent)
    m['cmd_retx_frames_total'] = sum(retx_sent)

    # --- [RX-SACK] Detected (... ldpc=YES|NO ...), bitmap: 0 1 1 ... ---------
    #   arq_common.cc:3875-3879. Watch for ldpc=YES with a provably-wrong
    #   bitmap (the deferred "Candidate B" suffix-quality gate question).
    rx_sack = []
    for mo in re.finditer(
            r'\[RX-SACK\]\s+Detected\s+\(matched=(\d+),\s+metric=([\d.]+),'
            r'\s+ack_xcheck=(\d+),\s+ldpc=(YES|NO)\),\s+bitmap:([0-9 ]*)',
            cmd_text):
        bm = [int(x) for x in mo.group(5).split()]
        rx_sack.append({'matched': int(mo.group(1)), 'metric': float(mo.group(2)),
                        'ack_xcheck': int(mo.group(3)), 'ldpc': mo.group(4),
                        'bitmap': bm})
    m['rx_sack_detected'] = rx_sack
    m['rx_sack_ldpc_yes'] = sum(1 for e in rx_sack if e['ldpc'] == 'YES')
    m['rx_sack_ldpc_no']  = sum(1 for e in rx_sack if e['ldpc'] == 'NO')
    # ldpc=NO -> full-batch retransmit (hard fallback skipped) — e076823 path
    m['rx_sack_ldpc_no_fullbatch'] = len(
        re.findall(r'\[RX-SACK\]\s+ldpc=NO\s+->\s+full-batch retransmit', cmd_text))

    # --- RSP [TX-SACK] Sending SACK pattern (batch=N, received: 0 1 1 ...) ---
    #   arq_common.cc:3535-3538. The bitmap RSP ACTUALLY transmitted. Cross-
    #   checking this against CMD's [RX-SACK] Detected bitmap is how an LDPC
    #   miscorrection (ldpc=YES but wrong bitmap) is caught provably.
    tx_sack = []
    for mo in re.finditer(
            r'\[TX-SACK\]\s+Sending SACK pattern\s+\(batch=(\d+),\s+received:([0-9 ]*)\)',
            rsp_text):
        tx_sack.append({'batch': int(mo.group(1)),
                        'bitmap': [int(x) for x in mo.group(2).split()]})
    m['tx_sack_sent'] = tx_sack
    m['n_tx_sack_sent'] = len(tx_sack)
    # LDPC-miscorrection test (the deferred "Candidate B" suffix-quality-gate
    # question). A *provable* miscorrection = CMD decodes a bitmap (ldpc=YES)
    # that RSP *never sent*. We CANNOT use k-th<->k-th alignment: RSP re-sends
    # a SACK whenever it gets no retransmit back (turnaround collision / loss),
    # so there are routinely MORE [TX-SACK] events than [RX-SACK] events and
    # the k-th pair is not a true correspondence. The robust, alignment-free
    # test: a CMD ldpc=YES bitmap is CORRECT iff it equals SOME [TX-SACK]
    # bitmap RSP emitted in this run; it is a miscorrection iff it matches
    # NONE of them. (RSP's [TX-SACK] log is the ground truth of what was
    # actually keyed onto the channel — arq_common.cc:3535.)
    tx_bitmaps = [tuple(t['bitmap']) for t in tx_sack]
    tx_bitmap_set = set(tx_bitmaps)
    miscorrections = []
    for k, rxev in enumerate(rx_sack):
        if rxev['ldpc'] != 'YES':
            continue
        rxbm = tuple(rxev['bitmap'])
        if not rxbm:
            continue
        if rxbm not in tx_bitmap_set:
            # CMD's ldpc=YES decode matches NO bitmap RSP ever sent.
            # Compute the closest RSP-sent bitmap for diagnostics.
            best = None
            best_h = 1e9
            for tb in tx_bitmaps:
                n = min(len(tb), len(rxbm))
                h = sum(1 for a, b in zip(tb[:n], rxbm[:n]) if a != b) \
                    + abs(len(tb) - len(rxbm))
                if h < best_h:
                    best_h, best = h, tb
            miscorrections.append({
                'index': k, 'metric': rxev['metric'],
                'cmd_decoded': list(rxbm),
                'closest_rsp_sent': list(best) if best else None,
                'closest_hamming': best_h if best else None,
            })
    m['ldpc_miscorrections'] = miscorrections
    m['n_ldpc_miscorrections'] = len(miscorrections)
    # Milder, distinct observation: CMD's ldpc=YES decode is a VALID RSP
    # bitmap but NOT the most-recent one (i.e. CMD acted on a stale SACK
    # because it missed RSP's later re-send). Not a miscorrection — the
    # decode was correct — but worth counting for the turnaround picture.
    stale = 0
    for rxev in rx_sack:
        if rxev['ldpc'] != 'YES':
            continue
        rxbm = tuple(rxev['bitmap'])
        if rxbm in tx_bitmap_set and tx_bitmaps and rxbm != tx_bitmaps[-1]:
            stale += 1
    m['rx_sack_stale_but_valid'] = stale

    # --- batch_data_delivered throughput lines (if present) ------------------
    bd = []
    for mo in re.finditer(r'batch_data_delivered.*?=\s*([\d.]+)\s*bps', cmd_text):
        bd.append(float(mo.group(1)))
    if bd:
        m['batch_data_delivered_bps'] = bd

    # --- batch cycle timing from [T] instrumentation ------------------------
    #   arq_commander.cc: [T] cmd_batch_tx_start abs_ms=N batch=K nframes=F
    #   A full batch cycle = consecutive cmd_batch_tx_start abs_ms deltas.
    #   This is the headline mechanism number: SACK should have FEWER, and
    #   on a lossy channel SHORTER, cycles than no-SACK per delivered byte.
    starts = []
    for mo in re.finditer(r'\[T\]\s+cmd_batch_tx_start\s+abs_ms=(\d+)\s+batch=(\d+)',
                          cmd_text):
        starts.append({'abs_ms': int(mo.group(1)), 'batch': int(mo.group(2))})
    m['n_batch_tx_start'] = len(starts)
    if len(starts) >= 2:
        cycles = [starts[i + 1]['abs_ms'] - starts[i]['abs_ms']
                  for i in range(len(starts) - 1)]
        # drop the first cycle (includes connection-setup tail) if >1 cycle
        body = cycles[1:] if len(cycles) > 1 else cycles
        m['batch_cycle_ms_mean'] = round(sum(body) / len(body), 1) if body else None
        m['batch_cycle_ms_min']  = min(body) if body else None
        m['batch_cycle_ms_max']  = max(body) if body else None
        m['batch_cycle_ms_all']  = cycles
    # done events too — for batch TX (audio) duration vs full cycle
    dones = [int(x) for x in re.findall(
        r'\[T\]\s+cmd_batch_tx_done\s+abs_ms=(\d+)', cmd_text)]
    m['n_batch_tx_done'] = len(dones)
    m['n_partial_batches'] = len(cmd_sack)

    # --- gearshift sanity: there should be NONE (fixed config) ---------------
    m['gearshift_lines'] = len(re.findall(r'GEARSHIFT|SUPERSHIFT', cmd_text))

    # --- success rate (last reported) ----------------------------------------
    sr = re.findall(r'last_transmission_block_success_rate=\s*(\d+)', cmd_text)
    if sr:
        m['last_block_success_rate_pct'] = int(sr[-1])

    # --- Design A v2 evidence (Step 13 win-test only) -----------------------
    # Negotiation surface
    m['cap_sack_v2_negotiated'] = (
        re.search(r'\[SACK-V2\]\s+enabled', cmd_text) is not None
        or re.search(r'\[SACK-V2\]\s+enabled', rsp_text) is not None)
    # Axis-1 / Axis-2 / Axis-3 move counts (CMD-side)
    m['policy_move_axis1'] = len(re.findall(r'\[POLICY-MOVE\]\s+axis=1\b', cmd_text))
    m['policy_move_axis2'] = len(re.findall(r'\[POLICY-MOVE\]\s+axis=2\b', cmd_text))
    m['policy_move_axis3'] = len(re.findall(r'\[POLICY-MOVE\]\s+axis=3\b', cmd_text))
    m['policy_supremacy_events'] = len(re.findall(r'\[POLICY-SUPREMACY\]', cmd_text))
    m['axis2_ceiling_set'] = len(re.findall(r'\[POLICY-AXIS2-CEILING\]\s+down-move', cmd_text))
    m['axis2_ceiling_vetoed'] = len(re.findall(r'\[POLICY-AXIS2-CEILING\]\s+up-move VETOED', cmd_text))
    # Mixed-batch + prev-buffer mechanism
    m['cmd_v2_mixbatch_count'] = len(re.findall(r'\[CMD-V2-MIXBATCH\]\s+TX batch', cmd_text))
    m['rsp_v2_prev_bump_count'] = len(re.findall(r'\[RSP-V2-PREV-BUMP\]', rsp_text))
    m['rsp_v2_prev_delivered_count'] = len(re.findall(r'\[RSP-V2-PREV-DELIVERED\]', rsp_text))
    m['rsp_v2_prev_stale_count'] = len(re.findall(r'\[RSP-V2-PREV-STALE\]', rsp_text))
    # SACK_RSP OFDM frame surface (v2)
    m['rsp_tx_sack_v2_count'] = len(re.findall(r'\[TX-SACK-V2\]', rsp_text))
    m['cmd_rx_sack_v2_count'] = len(re.findall(r'\[CMD-SACK-V2\]\s+decoded', cmd_text))
    m['cmd_sack_v2_crc_fail_count'] = len(re.findall(r'\[CMD-SACK-V2-CRC-FAIL\]', cmd_text))
    # batch_seq_id mis-slotting (Step 4 + Step 8a discard branches)
    m['rsp_v2_drop_count'] = len(re.findall(r'\[RSP-V2-DROP\]', rsp_text))
    # SET_LINK_PARAMS round-trips (Axis-2 + Axis-3 wire)
    m['cmd_link_params_tx_count'] = len(re.findall(r'\[CMD-LINK-PARAMS\]\s+SET_LINK_PARAMS TX', cmd_text))
    m['cmd_link_params_acked_count'] = len(re.findall(r'\[CMD-LINK-PARAMS-ACKED\]', cmd_text))
    m['rsp_link_params_applied_count'] = len(re.findall(r'\[RSP-LINK-PARAMS\]\s+APPLIED', rsp_text))
    # axis 3 mode transitions captured
    m['axis3_on_to_probe'] = len(re.findall(r'POLICY-MOVE\]\s+axis=3\s+from=ON\s+to=PROBE', cmd_text))
    m['axis3_probe_to_off'] = len(re.findall(r'POLICY-MOVE\]\s+axis=3\s+from=PROBE\s+to=OFF', cmd_text))
    m['axis3_probe_to_on']  = len(re.findall(r'POLICY-MOVE\]\s+axis=3\s+from=PROBE\s+to=ON', cmd_text))
    m['axis3_off_to_probe'] = len(re.findall(r'POLICY-MOVE\]\s+axis=3\s+from=OFF\s+to=PROBE', cmd_text))

    return m


# ── One A/B cell run ──────────────────────────────────────────────────────────
def run_one(bs, lid, point_name, channel_cmds, cfg_label, cfg_id, is_nb,
            mode, run_idx, duration_s, payload, out_dir, settle_s=30,
            sack_rx_trace=False, gearshift=False, compress='off',
            max_config=None, optimizer_disabled=False,
            mercury_log_path=None):
    """One run for a single (channel point, mode, run index) cell.

    mode: 'sack' -> pass --enable-sack to both ; 'nosack' -> nothing extra ;
          'sackv2' -> pass `--enable-sack --enable-sack-v2` to both (Design A).
    optimizer_disabled: when True, append --no-optimizer to mercury's flags so
          the Phase 3c effective-rate optimizer is fully inert (no table load,
          no per-batch evaluation). Used by tools/effective_rate_calibrate.py
          so calibration sweeps measure each fixed config without the
          optimizer trying to switch configs mid-run.
    Assumes the butler lease `lid` is already held and the channel `point`
    has ALREADY been programmed by the caller (we program once per point and
    interleave runs under it to avoid serial-command churn between A/B pairs).
    """
    if mode == 'sack':
        sack_flag = '--enable-sack'
    elif mode == 'sackv2':
        sack_flag = '--enable-sack --enable-sack-v2'
    else:
        # mode='nosack' — explicit opt-out. Until 2026-05-19 this was just
        # an empty flag, which relied on SACK being default-OFF in mercury.
        # SACK is now default-ON; without --no-sack the 'nosack' label would
        # silently run with SACK on and pollute A/B comparisons.
        sack_flag = '--no-sack'
    label = f'{point_name}_{cfg_label}_{mode}_r{run_idx}'
    result = {
        'point': point_name, 'channel_cmds': channel_cmds,
        'config': cfg_label, 'config_id': cfg_id, 'is_nb': is_nb,
        'mode': mode, 'sack': mode == 'sack', 'run': run_idx,
        'duration_s': 0, 'bps': 0, 'rx_bytes': 0, 'tx_bytes': 0,
        'rx_bytes_at_settle': 0, 'rx_bytes_measured': 0,
        'settle_s': settle_s, 'measured_s': 0,
        'connected': False, 'error': None,
        'byte_exact': False, 'mismatch_bytes': 0,
        'first_mismatch_offset': None,
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'cmd_log': None, 'rsp_log': None,
    }

    mode_flags = '-Q 0 -M nb' if is_nb else '-Q 0 -M auto'
    # Same flags as phase0_baseline.py pi_run_one + the SACK toggle.
    # Gearshift OFF (default): -F off, --skip-turbo-reverse, fixed -s.
    # Gearshift ON: -g enabled, no --skip-turbo-reverse, -F honors --compress.
    if gearshift:
        gear_flag = '-g'
        # KEEP --skip-turbo-reverse even with -g: the v1 gearshift panel
        # found that REVERSE-probe SWITCH_ROLE fires in single-sender
        # benchmark mode (RSP has no data, becomes a silent idle sender)
        # which then triggers phantom BREAK detection at MFSK-pattern
        # metric=16/16 and a forced fallback to CFG0. Both bugs are
        # latent; the harness avoids them by keeping turbo-reverse off.
        skip_flag = '--skip-turbo-reverse'
        max_cfg_flag = f'--max-config {max_config}' if max_config is not None else ''
    else:
        gear_flag = ''
        skip_flag = '--skip-turbo-reverse'
        max_cfg_flag = ''
    # Honor caller's compress= regardless of gearshift state. Callers that
    # want apples-to-apples raw-PHY (Q-table calibration, SACK A/B gain
    # measurement) pass compress='off' explicitly. Callers that want
    # production-rate measurement (Phase 4 benchmark) pass compress='on'.
    # The historical behavior of forcing compress=off when gearshift=False
    # was a silent footgun — it made the calibration table look like raw PHY
    # without making that intent explicit at the call site.
    compress_flag = f'-F {compress}'
    opt_flag = '--no-optimizer' if optimizer_disabled else ''
    # MERCURY_EXTRA_FLAGS: caller-supplied extra mercury CLI flags appended to
    # BOTH the CMD and RSP launches (identical flags for both ends — see the
    # two nohup launches below). Default unset => byte-identical to prior
    # behavior. Used by the bessel-I0 (--mfsk-demap=...) and mini-Moose floor
    # A/Bs to toggle a runtime PHY flag on both demodulators (2026-05-31).
    extra_flags = os.environ.get('MERCURY_EXTRA_FLAGS', '').strip()
    flags = (f'-m ARQ -x alsa -i {PI_AUDIO_DEV} -o {PI_AUDIO_DEV} '
             f'--rx-channel 1 -s {cfg_id} {mode_flags} '
             f'-n -v {compress_flag} {skip_flag} {gear_flag} {max_cfg_flag} '
             f'{sack_flag} {opt_flag} {extra_flags}').strip()
    # collapse multiple spaces
    flags = ' '.join(flags.split())

    ctrl_cmd = ctrl_rsp = None
    stop = threading.Event()
    try:
        # 1. clean slate
        for rpi in ('rpi1', 'rpi2'):
            clean, reply = graceful_stop_mercury(bs, lid, rpi)
            if not clean:
                raise RuntimeError(f'preflight cleanup failed on {rpi}: {reply[:160]}')
        # truncate prior logs so this run's log is THIS run only
        b_send(bs, f'SSH {lid} rpi1 : > {RSP_LOG}; echo done')
        b_send(bs, f'SSH {lid} rpi2 : > {CMD_LOG}; echo done')

        # 2. audio levels
        for rpi in ('rpi1', 'rpi2'):
            b_send(bs, f'AUDIO_SETUP {lid} {rpi}', t=25)

        # 3. start Mercury — responder first
        # nohup takes a command, NOT a shell assignment, so use `env VAR=v cmd`.
        # MERCURY_RATE_TABLE points mercury at the calibration table that lives
        # in the Pi's mercury-dev/ workspace (deploy script puts it there).
        # Without this, mercury launches from $HOME and looks for
        # mercury/effective_rate_table.json / effective_rate_table.json
        # relative to CWD, neither of which exists → optimizer disabled.
        rate_table_env = 'MERCURY_RATE_TABLE=~/mercury-dev/effective_rate_table.json '
        sack_rx_trace_env = 'MERCURY_SACK_RX_TRACE=1 ' if sack_rx_trace else ''
        # Propagate drift-instr soak env vars from caller. Set on the host
        # ("MERCURY_DRIFT_INSTR=1 python tools/effective_rate_calibrate.py ...")
        # to enable [DRIFT-INSTR] periodic logging on the Pis during a soak.
        # See mercury/fact-documents/pi-audio-drift-audit.md.
        drift_env = ''
        if os.environ.get('MERCURY_DRIFT_INSTR'):
            drift_env += 'MERCURY_DRIFT_INSTR=1 '
        if os.environ.get('MERCURY_HAIL_POLL'):
            drift_env += 'MERCURY_HAIL_POLL=1 '
        env_prefix = f'env {rate_table_env}{sack_rx_trace_env}{drift_env}'
        # Optional CMD-side mercury --log: when callers (e.g. optimizer_smoke)
        # need timestamped lines (Mercury's `--log` writes [HH:MM:SS.ms]
        # prefixes), they request it here. Stdout redirect alone has no
        # timestamps, so time-based criteria in downstream parsers silently
        # degrade without this. Path is on the Pi (where mercury runs).
        cmd_log_flag = f'--log {mercury_log_path}' if mercury_log_path else ''
        b_send(bs, f'SSH {lid} rpi1 nohup {env_prefix}{PI_MERCURY} {flags} '
                   f'-p {PI_RSP_PORT} > {RSP_LOG} 2>&1 &', t=15)
        time.sleep(4)
        b_send(bs, f'SSH {lid} rpi2 nohup {env_prefix}{PI_MERCURY} {flags} '
                   f'{cmd_log_flag} '
                   f'-p {PI_CMD_PORT} > {CMD_LOG} 2>&1 &', t=15)
        time.sleep(5)

        # 4. control sockets
        ctrl_rsp = tcp_connect_retry(PI_RSP_HOST, PI_RSP_PORT, retries=20)
        time.sleep(0.5)
        tcp_send_line(ctrl_rsp, 'MYCALL TESTB')
        time.sleep(0.3); tcp_recv_drain(ctrl_rsp)
        tcp_send_line(ctrl_rsp, 'LISTEN ON')
        time.sleep(0.3); tcp_recv_drain(ctrl_rsp)

        ctrl_cmd = tcp_connect_retry(PI_CMD_HOST, PI_CMD_PORT, retries=20)
        time.sleep(0.5)
        tcp_send_line(ctrl_cmd, 'MYCALL TESTA')
        time.sleep(0.3); tcp_recv_drain(ctrl_cmd)

        # 5. RX + TX threads
        rx = {'bytes': 0}
        rx_t = threading.Thread(target=receiver_thread,
                                args=(PI_RSP_HOST, PI_RSP_PORT + 1, rx, stop,
                                      payload))
        rx_t.daemon = True; rx_t.start()
        tx_res = {'tx_bytes': 0}
        tx_t = threading.Thread(target=sender_thread,
                                args=(PI_CMD_HOST, PI_CMD_PORT + 1, payload, stop, tx_res))
        tx_t.daemon = True; tx_t.start()
        time.sleep(2)

        # 6. connect
        # CONNECT wait. Default 120 s (unchanged for OFDM/ROBUST_0 calibration).
        # The deep baud-scaled ULTRA rungs (K=8: one CONNECT ctrl-suffix frame is
        # ~14 s airtime, multi-stage handshake) need a longer window — the
        # DEEP-ULTRA HW validation sets MERCURY_CONNECT_TIMEOUT=240. Env-gated so
        # the calibrator's normal use is byte-identical.
        connect_timeout = int(os.environ.get('MERCURY_CONNECT_TIMEOUT', '120'))
        tcp_send_line(ctrl_cmd, 'CONNECT TESTA TESTB')
        buf = tcp_recv_until(ctrl_cmd, 'CONNECTED', timeout=connect_timeout)
        if 'CONNECTED' not in buf or 'DISCONNECTED' in buf:
            result['error'] = f'connect_failed: {buf[:160]!r}'
            stop.set()
            rx_t.join(timeout=3); tx_t.join(timeout=3)
            return result
        result['connected'] = True
        b_send(bs, f'EXTEND {lid} 600')

        # 7. measurement window
        #    bps is measured over (duration - settle_s) starting from
        #    settle_s. The sackv2 Axis-2 ring-clean ramp (8 good batches
        #    per up-step from start=25) doesn't reach steady-state for
        #    30-90 s at CFG15, so the first --settle-s seconds are
        #    excluded from the throughput number. rx_bytes_at_settle is
        #    sampled at the boundary for audit.
        t0 = time.time()
        # Sample bytes at the settle boundary so we measure steady-state only.
        # Sleep no longer than the full duration in case settle_s >= duration_s.
        settle_sleep = min(settle_s, duration_s) if settle_s > 0 else 0
        if settle_sleep > 0:
            time.sleep(settle_sleep)
        bytes_at_settle = rx.get('bytes', 0)
        extend_at = 150
        while time.time() - t0 < duration_s:
            time.sleep(5)
            if time.time() - t0 > extend_at:
                b_send(bs, f'EXTEND {lid} 300')
                extend_at += 150
        elapsed = time.time() - t0
        measured_s = max(0.0, elapsed - settle_s)
        stop.set()
        rx_t.join(timeout=3); tx_t.join(timeout=3)

        delivered_total    = rx.get('bytes', 0)
        delivered_measured = max(0, delivered_total - bytes_at_settle)
        result['rx_bytes']             = delivered_total
        result['rx_bytes_at_settle']   = bytes_at_settle
        result['rx_bytes_measured']    = delivered_measured
        result['tx_bytes']             = tx_res.get('tx_bytes', 0)
        result['byte_exact']            = rx.get('byte_exact', False)
        result['mismatch_bytes']        = rx.get('mismatch_bytes', 0)
        result['first_mismatch_offset'] = rx.get('first_mismatch_offset')
        result['received_sha256']       = rx.get('received_sha256')
        result['expected_sha256']       = rx.get('expected_sha256')
        if delivered_total > 0 and not result['byte_exact']:
            result['error'] = 'byte_integrity_failed'
        result['duration_s']           = round(elapsed, 1)
        result['settle_s']             = settle_s
        result['measured_s']           = round(measured_s, 1)
        result['bps'] = (round((delivered_measured * 8) / measured_s, 1)
                        if measured_s > 0 else 0)

        try:
            tcp_send_line(ctrl_cmd, 'DISCONNECT')
        except Exception:
            pass
        time.sleep(3)
    except Exception as e:
        result['error'] = f'exception: {type(e).__name__}: {e}'
    finally:
        for s in (ctrl_cmd, ctrl_rsp):
            try:
                if s:
                    s.close()
            except Exception:
                pass
        # Stop Mercury BEFORE pulling logs (avoids log-rotation race).  Never
        # hard-kill a process that may still own the Fe-Pi ALSA device.
        for rpi in ('rpi1', 'rpi2'):
            clean, reply = graceful_stop_mercury(bs, lid, rpi)
            if not clean:
                result['cleanup_error'] = f'{rpi}: {reply[:160]}'
                result['error'] = result.get('error') or 'graceful_cleanup_failed'

        # 8. pull logs — deterministic per-run filename, space-free staging.
        #    The butler DOWNLOAD splits args on whitespace, so a destination
        #    with spaces (the workspace share) is truncated. Stage in a
        #    space-free tmp dir then move. (timing_pull_pi_logs.py pattern.)
        os.makedirs(out_dir, exist_ok=True)
        tmpdir = tempfile.mkdtemp(prefix='sack_lossy_pull_')
        try:
            for rpi, remote, tag in (('rpi2', CMD_LOG, 'cmd'),
                                     ('rpi1', RSP_LOG, 'rsp')):
                fname = f'sack_lossy_{label}_{tag}.log'
                staged = os.path.join(tmpdir, fname)
                local  = os.path.join(out_dir, fname)
                r = b_send(bs, f'DOWNLOAD {lid} {rpi} {remote} {staged}', t=90)
                if r.startswith('OK') and os.path.exists(staged):
                    shutil.move(staged, local)
                    result[f'{tag}_log'] = local
                else:
                    result[f'{tag}_log_error'] = r[:100]
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)

        # 9. parse the mechanism evidence from the freshly-pulled logs
        cmd_text = rsp_text = ''
        if result.get('cmd_log') and os.path.exists(result['cmd_log']):
            with open(result['cmd_log'], 'r', encoding='utf-8', errors='replace') as f:
                cmd_text = f.read()
        if result.get('rsp_log') and os.path.exists(result['rsp_log']):
            with open(result['rsp_log'], 'r', encoding='utf-8', errors='replace') as f:
                rsp_text = f.read()
        try:
            result['mechanism'] = parse_mercury_logs(cmd_text, rsp_text)
        except Exception as e:
            result['mechanism'] = {'parse_error': str(e)}
        # confirm SACK negotiation actually matches the requested mode
        result['sack_negotiated'] = ('--enable-sack' in cmd_text or
                                     'CAP_SACK' in cmd_text and 'opt-in SACK' in cmd_text)
        result['enable_sack_flag_in_log'] = ('opt-in SACK negotiation' in cmd_text)

    return result


def stats(values):
    if not values:
        return {'n': 0, 'mean': 0, 'sigma': 0, 'min': 0, 'max': 0}
    n = len(values)
    mean = sum(values) / n
    sigma = (sum((v - mean) ** 2 for v in values) / n) ** 0.5 if n > 1 else 0.0
    return {'n': n, 'mean': round(mean, 1), 'sigma': round(sigma, 1),
            'min': round(min(values), 1), 'max': round(max(values), 1)}


def valid_run_bps(run):
    """Return a cell's bps only when positive delivery is byte-exact."""
    bps = run.get('bps', 0)
    return bps if bps > 0 and run.get('byte_exact') else None


def main():
    ap = argparse.ArgumentParser(description='SACK lossy-channel A/B sweep')
    ap.add_argument('--out', required=True, help='output JSON path')
    ap.add_argument('--config', default='WB_CFG15', choices=list(CONFIGS),
                    help='fixed config (no gearshift). default WB_CFG15')
    ap.add_argument('--points', default='clean,wgn17,wgn14,mpm15',
                    help='comma-separated channel points (see CHANNEL_POINTS)')
    ap.add_argument('--runs', type=int, default=3,
                    help='runs per (point,mode) cell. default 3 (upgrade to 5)')
    ap.add_argument('--duration', type=int, default=90,
                    help='measurement window seconds per run. default 90')
    ap.add_argument('--settle-s', type=int, default=30,
                    help='exclude first N seconds from bps measurement '
                         '(default 30 — sackv2 Axis-2 batch-size ramp '
                         'settles by then). bps is computed over '
                         '(duration - settle_s) starting from settle_s. '
                         'rx_bytes_at_settle / rx_bytes_measured / measured_s '
                         'are recorded per-run for audit.')
    ap.add_argument('--modes', default='sack,nosack',
                    help='comma-separated modes to run interleaved. '
                         'default "sack,nosack" (A/B). use "sack" alone for '
                         'loss-cliff calibration.')
    ap.add_argument('--payload', default=os.path.join(WORKSPACE, 'pg84.txt'))
    ap.add_argument('--sack-rx-trace', action='store_true',
                    help='prefix mercury launches with MERCURY_SACK_RX_TRACE=1 '
                         'to enable [SACK-RX-TRACE] diagnostic logging in cmd/rsp logs.')
    ap.add_argument('--gearshift', action='store_true',
                    help='Enable adaptive gearshift (-g). When set, drops '
                         '`-F off --skip-turbo-reverse` so the modem can '
                         'turbo up/down across the SNR range. --config '
                         'becomes the STARTING config; modem moves from there.')
    ap.add_argument('--compress', default='off', choices=['on','off','auto'],
                    help='Compression mode passed as -F <mode>. default off '
                         '(legacy harness behavior). use "on" for text payloads '
                         'like pg84.txt to engage PPMd+zstd streaming compression.')
    ap.add_argument('--max-config', type=int, default=None,
                    help='Hard ceiling on turboshift (0-17). Default: no cap.')
    args = ap.parse_args()

    modes = [m.strip() for m in args.modes.split(',') if m.strip()]
    for m in modes:
        if m not in ('sack', 'nosack', 'sackv2'):
            print(f'ERROR: unknown mode {m!r} (valid: sack, nosack, sackv2)')
            sys.exit(1)

    cfg_label = args.config
    cfg_id, is_nb = CONFIGS[cfg_label]
    points = [p.strip() for p in args.points.split(',') if p.strip()]
    for p in points:
        if p not in CHANNEL_POINTS:
            print(f'ERROR: unknown channel point {p!r}. '
                  f'Valid: {",".join(CHANNEL_POINTS)}')
            sys.exit(1)

    payload = (open(args.payload, 'rb').read() if os.path.exists(args.payload)
               else bytes(range(256)) * 1024)
    print(f'Payload: {len(payload)} bytes ({args.payload})')

    out_dir = os.path.splitext(args.out)[0] + '_logs'
    out = {
        'test': 'SACK lossy-channel A/B sweep',
        'config': cfg_label, 'config_id': cfg_id,
        'gearshift': args.gearshift,
        'compress': args.compress,
        'max_config': args.max_config,
        'mercury_head': '4dd8ffb',
        'sack_fixes': ['f2dbf34', 'e076823'],
        'points': points, 'runs_per_cell': args.runs,
        'duration_s': args.duration,
        'settle_s': args.settle_s,
        'measured_s_per_run': max(0, args.duration - args.settle_s),
        'started': time.strftime('%Y%m%d_%H%M%S'),
        'channel_definitions': {p: CHANNEL_POINTS[p] for p in points},
        'runs': [], 'summary': {},
    }

    def flush():
        with open(args.out, 'w') as f:
            json.dump(out, f, indent=2)

    for point in points:
        channel_cmds = CHANNEL_POINTS[point]
        print(f'\n{"="*70}\n  CHANNEL POINT: {point}  ->  {channel_cmds}\n{"="*70}')
        # Acquire ONE lease for this whole point (program channel once, then
        # interleave A/B runs under it). Re-lock per point to keep leases short.
        bs, lid = b_lock(f'sack_lossy_{point}', 600)
        try:
            # clean slate + program channel
            for rpi in ('rpi1', 'rpi2'):
                clean, reply = graceful_stop_mercury(bs, lid, rpi)
                if not clean:
                    raise RuntimeError(
                        f'point cleanup failed on {rpi}: {reply[:160]}')
            for c in channel_cmds:
                r = b_send(bs, f'IONOS {lid} {c}', t=12)
                print(f'  IONOS {c} -> {r[:40]}')
                time.sleep(0.2)

            # INTERLEAVED schedule: for each run index, all requested modes
            # in turn (sack, nosack, sack, nosack, ...) so channel drift
            # biases neither mode. With --modes sack this is calibration.
            schedule = []
            for r in range(1, args.runs + 1):
                for mode in modes:
                    schedule.append((mode, r))

            for mode, run_idx in schedule:
                print(f'\n  --- {point} / {cfg_label} / {mode} run {run_idx}/{args.runs} ---')
                # keep lease fresh before each (each run can take ~2-3 min)
                b_send(bs, f'EXTEND {lid} 600')
                res = run_one(bs, lid, point, channel_cmds, cfg_label, cfg_id,
                              is_nb, mode, run_idx, args.duration, payload, out_dir,
                              settle_s=args.settle_s,
                              sack_rx_trace=args.sack_rx_trace,
                              gearshift=args.gearshift,
                              compress=args.compress,
                              max_config=args.max_config)
                mech = res.get('mechanism', {})
                print(f'    bps={res["bps"]} rx={res.get("rx_bytes",0)} '
                      f'exact={res.get("byte_exact")} conn={res["connected"]} '
                      f'err={res.get("error")}')
                print(f'    mechanism: sack_events={mech.get("n_cmd_sack_events")} '
                      f'retx_bursts={mech.get("cmd_retx_bursts")} '
                      f'retx_frames={mech.get("cmd_retx_frames_total")} '
                      f'nReSent={mech.get("cmd_nReSent_data")} '
                      f'nSent={mech.get("cmd_nSent_data")} '
                      f'ldpc_no={mech.get("rx_sack_ldpc_no")} '
                      f'ldpc_yes={mech.get("rx_sack_ldpc_yes")} '
                      f'miscorrect={mech.get("n_ldpc_miscorrections")} '
                      f'sack_loss_mean={mech.get("sack_per_batch_loss_mean")} '
                      f'gearshift_lines={mech.get("gearshift_lines")}')
                out['runs'].append(res)
                flush()
        finally:
            try:
                for rpi in ('rpi1', 'rpi2'):
                    graceful_stop_mercury(bs, lid, rpi)
                b_send(bs, f'UNLOCK {lid}')
                bs.close()
            except Exception:
                pass

        # per-point summary
        for mode in modes:
            cell = [r for r in out['runs']
                    if r['point'] == point and r['mode'] == mode]
            valid = [bps for r in cell if (bps := valid_run_bps(r)) is not None]
            s = stats(valid)
            s['failures'] = sum(
                1 for r in cell if r['bps'] == 0 or not r.get('byte_exact'))
            s['corrupt_runs'] = sum(
                1 for r in cell if (r.get('mismatch_bytes') or 0) > 0)
            # aggregate mechanism
            s['total_retx_frames'] = sum(
                (r.get('mechanism', {}).get('cmd_retx_frames_total') or 0)
                for r in cell)
            s['total_nReSent']     = sum(
                (r.get('mechanism', {}).get('cmd_nReSent_data') or 0)
                for r in cell if r.get('mechanism', {}).get('cmd_nReSent_data'))
            s['total_sack_events'] = sum(
                (r.get('mechanism', {}).get('n_cmd_sack_events') or 0)
                for r in cell)
            s['ldpc_no_total']  = sum(
                (r.get('mechanism', {}).get('rx_sack_ldpc_no') or 0) for r in cell)
            s['ldpc_yes_total'] = sum(
                (r.get('mechanism', {}).get('rx_sack_ldpc_yes') or 0) for r in cell)
            s['ldpc_miscorrections_total'] = sum(
                (r.get('mechanism', {}).get('n_ldpc_miscorrections') or 0) for r in cell)
            s['total_nSent'] = sum(
                (r.get('mechanism', {}).get('cmd_nSent_data') or 0)
                for r in cell if r.get('mechanism', {}).get('cmd_nSent_data'))
            # frame-level retransmit rate: nReSent / nSent (apples-to-apples
            # across both modes — the universal "wasted-frame" metric)
            s['retx_rate'] = (round(s['total_nReSent'] / s['total_nSent'], 4)
                              if s.get('total_nSent') else None)
            out['summary'][f'{point}_{mode}'] = s
        flush()

        # crossover line for this point
        ss = out['summary'].get(f'{point}_sack', {})
        sn = out['summary'].get(f'{point}_nosack', {})
        if ss.get('mean') and sn.get('mean'):
            delta = ss['mean'] - sn['mean']
            pct = 100.0 * delta / sn['mean'] if sn['mean'] else 0
            print(f'\n  >>> {point}: SACK={ss["mean"]} bps  noSACK={sn["mean"]} bps  '
                  f'delta={delta:+.1f} bps ({pct:+.1f}%)  '
                  f'{"SACK WINS" if delta > 0 else "no-SACK wins"}')

    out['finished'] = time.strftime('%Y%m%d_%H%M%S')
    flush()

    print(f'\n{"="*70}\n  SUMMARY — {cfg_label}, no gearshift\n{"="*70}')
    if modes == ['sack'] or modes == ['nosack']:
        # single-mode (calibration) — print loss/retx instead of A/B delta
        m0 = modes[0]
        print(f'  {"point":<10} {"bps":>10} {"sigma":>8} {"sackloss":>10} '
              f'{"retx_rate":>10} {"sack_ev":>8} {"miscorr":>8}')
        for point in points:
            s = out['summary'].get(f'{point}_{m0}', {})
            losses = [(r.get('mechanism', {}).get('sack_per_batch_loss_mean'))
                      for r in out['runs']
                      if r['point'] == point and r['mode'] == m0
                      and r.get('mechanism', {}).get('sack_per_batch_loss_mean') is not None]
            lossm = round(sum(losses) / len(losses), 4) if losses else None
            print(f'  {point:<10} {s.get("mean",0):>10.1f} {s.get("sigma",0):>8.1f} '
                  f'{str(lossm):>10} {str(s.get("retx_rate")):>10} '
                  f'{s.get("total_sack_events",0):>8} '
                  f'{s.get("ldpc_miscorrections_total",0):>8}')
    else:
        print(f'  {"point":<10} {"SACK bps":>12} {"noSACK bps":>12} '
              f'{"delta":>10} {"verdict":>14}')
        for point in points:
            ss = out['summary'].get(f'{point}_sack', {})
            sn = out['summary'].get(f'{point}_nosack', {})
            sm = ss.get('mean', 0)
            nm = sn.get('mean', 0)
            d = sm - nm
            verdict = ('SACK wins' if d > 0 else
                       ('tie' if d == 0 else 'no-SACK wins'))
            print(f'  {point:<10} {sm:>12.1f} {nm:>12.1f} {d:>+10.1f} '
                  f'{verdict:>14}')
    print(f'\n  Full results: {args.out}')
    print(f'  Per-run logs: {out_dir}')


if __name__ == '__main__':
    main()
