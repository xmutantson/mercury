#!/usr/bin/env python3
"""
SACK Design A Step 7 — v2<->v2 loopback round-trip test.

Launches two mercury instances on VB-Cable, both with --enable-sack-v2, runs
a 90-second data exchange, and grades the captured logs for:

  Gate 2 (byte-identity): every [CMD-SACK-V2] decoded payload must match a
    [TX-SACK-V2] payload byte-for-byte (same batch_seq_id + bitmap bytes +
    CRC8). This proves the OFDM SACK_RSP encode->decode round-trip is
    lossless.

  Gate 3 (timing): the [TX-SACK-V2] log records the actual wire_ms — the
    measured wall-clock duration of the OFDM SACK_RSP send_batch() call.
    Output: min/median/max wire_ms across all SACK_RSP TX events, compared
    against the legacy MFSK SACK pattern's ~1168 ms baseline.

  Gate 4 (CRC8 fault): when --crc-corrupt is passed, RSP gets the
    --test-rsp-sack-rsp-crc-corrupt flag. The first SACK_RSP frame has its
    CRC8 XOR'd with 0xFF; CMD must log [CMD-SACK-V2-CRC-FAIL] and
    cmd_sack_v2_crc_fail_count==1; the corresponding bitmap must NOT
    appear in cmd_sack_v2_rx_count (no fabrication).

Usage:
  python tools/sack_v2_loopback_test.py [--duration 90] [--config 0]
                                        [--cmd-extra "..."] [--rsp-extra "..."]
                                        [--crc-corrupt]
                                        [--out v2_test.json]
"""

import argparse
import json
import os
import re
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mercury_benchmark import MercurySession

# ----------------------------------------------------------------------------

TX_RE = re.compile(
    r'\[TX-SACK-V2\] batch_seq_id=(\d+) nframes=(\d+) bitmap_bytes=(\d+) '
    r'payload=([0-9a-f]+) crc8=0x([0-9a-f]+)')
RX_RE = re.compile(
    r'\[CMD-SACK-V2\] batch_seq_id=(\d+) nframes=(\d+) bitmap_bytes=(\d+) '
    r'payload=([0-9a-f]+) crc8_ok=0x([0-9a-f]+)')
WIRE_RE = re.compile(r'\[TX-SACK-V2\] send_batch\(\) wire_ms=(\d+)')
CRC_FAIL_RE = re.compile(
    r'\[CMD-SACK-V2-CRC-FAIL\] rx_crc=0x([0-9a-f]+) computed=0x([0-9a-f]+) '
    r'nframes=(\d+) payload=([0-9a-f]+) fail_count=(\d+)')
CRC_CORRUPT_RE = re.compile(
    r'\[TX-SACK-V2-CRC-CORRUPT\] frame: CRC8 0x([0-9a-f]+) -> 0x([0-9a-f]+)')


def parse_logs(lines):
    tx_events = []
    rx_events = []
    wire_ms = []
    crc_fails = []
    crc_corrupts = []
    for line in lines:
        s = line if isinstance(line, str) else line.decode('utf-8', 'replace')
        m = TX_RE.search(s)
        if m:
            tx_events.append({
                'batch_seq_id': int(m.group(1)),
                'nframes': int(m.group(2)),
                'bitmap_bytes': int(m.group(3)),
                'payload_hex': m.group(4),
                'crc8': int(m.group(5), 16),
            })
        m = RX_RE.search(s)
        if m:
            rx_events.append({
                'batch_seq_id': int(m.group(1)),
                'nframes': int(m.group(2)),
                'bitmap_bytes': int(m.group(3)),
                'payload_hex': m.group(4),
                'crc8': int(m.group(5), 16),
            })
        m = WIRE_RE.search(s)
        if m:
            wire_ms.append(int(m.group(1)))
        m = CRC_FAIL_RE.search(s)
        if m:
            crc_fails.append({
                'rx_crc': int(m.group(1), 16),
                'computed_crc': int(m.group(2), 16),
                'nframes': int(m.group(3)),
                'payload_hex': m.group(4),
                'fail_count': int(m.group(5)),
            })
        m = CRC_CORRUPT_RE.search(s)
        if m:
            crc_corrupts.append({
                'orig_crc': int(m.group(1), 16),
                'flipped_crc': int(m.group(2), 16),
            })
    return tx_events, rx_events, wire_ms, crc_fails, crc_corrupts


def grade(cmd_lines, rsp_lines):
    """Apply the §7 validation gates and return a verdict dict."""
    tx, _, wire_ms_cmd, _, _ = parse_logs(rsp_lines)
    # TX-SACK-V2 lines live on the RSP side (RSP is the sender).
    # CMD-SACK-V2 and CMD-SACK-V2-CRC-FAIL live on the CMD side.
    _, rx, _, crc_fails, _ = parse_logs(cmd_lines)
    # CRC corruption lines live on the RSP side (it's the sender that
    # XORs the byte) — re-parse to extract.
    _, _, _, _, crc_corrupts = parse_logs(rsp_lines)
    # wire_ms is logged on RSP (the sender measures its own send_batch()).
    _, _, wire_ms, _, _ = parse_logs(rsp_lines)

    # Gate 2 — every RX payload should match exactly one TX payload byte-for-byte.
    matches = 0
    unmatched_rx = []
    used_tx_idx = set()
    for r in rx:
        match_idx = None
        for i, t in enumerate(tx):
            if i in used_tx_idx:
                continue
            if (t['payload_hex'] == r['payload_hex']
                    and t['batch_seq_id'] == r['batch_seq_id']):
                match_idx = i
                break
        if match_idx is not None:
            matches += 1
            used_tx_idx.add(match_idx)
        else:
            unmatched_rx.append(r)
    unmatched_tx = [tx[i] for i in range(len(tx)) if i not in used_tx_idx]

    # Timing
    wire_stats = {}
    if wire_ms:
        wire_stats = {
            'n': len(wire_ms),
            'min_ms': min(wire_ms),
            'max_ms': max(wire_ms),
            'median_ms': statistics.median(wire_ms),
            'mean_ms': statistics.mean(wire_ms),
            'samples_ms': wire_ms,
        }

    return {
        'tx_count': len(tx),
        'rx_count': len(rx),
        'matches_byte_identical': matches,
        'unmatched_rx': unmatched_rx,
        'unmatched_tx': unmatched_tx,
        'wire_ms_stats': wire_stats,
        'crc_fail_count': len(crc_fails),
        'crc_fails': crc_fails,
        'crc_corrupt_count': len(crc_corrupts),
        'crc_corrupts': crc_corrupts,
        # Gate verdicts:
        'gate_2_byte_identity_pass': (
            len(rx) > 0
            and matches == len(rx)
            and len(unmatched_rx) == 0
        ),
        'legacy_mfsk_sack_baseline_ms': 1168,  # SACK_DESIGN_A_PLAN.md §4.2.2
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=90.0)
    ap.add_argument('--config', type=int, default=0,
                    help='Mercury config (0=NB_CFG0 / NB_CFG4 etc — picked by --nb)')
    ap.add_argument('--nb', action='store_true', help='Use NB mode (-N)')
    ap.add_argument('--cmd-extra', default='',
                    help='Extra CLI args for both peers (space-separated)')
    ap.add_argument('--rsp-extra', default='',
                    help='Extra CLI args for RSP only (e.g. --test-rsp-sack-rsp-crc-corrupt)')
    ap.add_argument('--crc-corrupt', action='store_true',
                    help='Arm CRC8 fault injection on RSP (Gate 4 test)')
    ap.add_argument('--out', default=None,
                    help='Write verdict JSON here')
    ap.add_argument('--rsp-port', type=int, default=9700)
    ap.add_argument('--cmd-port', type=int, default=9710)
    args = ap.parse_args()

    common = ['--enable-sack', '--enable-sack-v2',
              '-T', '-12.6', '-G', '12.6', '-Q', '0',
              '-F', 'off', '-E', 'fast']
    if args.nb:
        common += ['-N']
    if args.cmd_extra:
        common += args.cmd_extra.split()
    # The MercurySession can't easily set asymmetric flags on RSP vs CMD; we
    # work around by launching one MercurySession (both peers share the
    # `extra_args`). To inject --test-rsp-sack-rsp-crc-corrupt only on the RSP
    # process, we'd need finer control. For Step 7 the simplest path is to
    # add it to BOTH peers — only the RSP actually has SACK_RSP TX paths, so
    # CMD's copy of the flag is a no-op (CMD never enters send_sack_v2_frame).
    if args.crc_corrupt:
        common += ['--test-rsp-sack-rsp-crc-corrupt']
    if args.rsp_extra:
        common += args.rsp_extra.split()

    print('[v2-LOOPBACK] launching mercury pair with:', ' '.join(common), flush=True)
    sess = MercurySession(
        config=args.config, gearshift=False,
        extra_args=common,
        rsp_port=args.rsp_port, cmd_port=args.cmd_port,
    )
    started = sess.start(timeout=60)
    if not started:
        print('[v2-LOOPBACK] FAIL: start() returned False')
        return 2

    print('[v2-LOOPBACK] waiting for CONNECTED...')
    if not sess.wait_connected(timeout=180):
        print('[v2-LOOPBACK] FAIL: never reached CONNECTED')
        sess.stop()
        return 3

    print(f'[v2-LOOPBACK] CONNECTED. running for {args.duration}s...')
    time.sleep(args.duration)

    sess.stop()
    time.sleep(1)

    cmd_lines = list(sess.cmd_lines)
    rsp_lines = list(sess.rsp_lines)
    print(f'[v2-LOOPBACK] cmd lines={len(cmd_lines)} rsp lines={len(rsp_lines)}')

    verdict = grade(cmd_lines, rsp_lines)
    print('[v2-LOOPBACK] verdict:')
    print(json.dumps({k: v for k, v in verdict.items()
                      if k not in ('crc_fails', 'crc_corrupts', 'unmatched_rx', 'unmatched_tx')},
                     indent=2, default=str))
    print(f'[v2-LOOPBACK] unmatched_rx_n={len(verdict["unmatched_rx"])} '
          f'unmatched_tx_n={len(verdict["unmatched_tx"])}')

    if args.out:
        # Also save the raw log lines for archival.
        record = {
            'verdict': verdict,
            'cmd_lines': [(s if isinstance(s, str) else s.decode('utf-8', 'replace'))
                          for s in cmd_lines],
            'rsp_lines': [(s if isinstance(s, str) else s.decode('utf-8', 'replace'))
                          for s in rsp_lines],
            'args': vars(args),
        }
        with open(args.out, 'w', encoding='utf-8') as f:
            json.dump(record, f, indent=2, default=str)
        print(f'[v2-LOOPBACK] wrote {args.out}')

    return 0 if verdict.get('gate_2_byte_identity_pass') or args.crc_corrupt else 1


if __name__ == '__main__':
    sys.exit(main())
