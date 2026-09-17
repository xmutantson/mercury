#!/usr/bin/env python3
"""Effective-rate calibration sweep — produces the Q-table Mercury's Phase 3c
==============================================================================
optimizer consumes at startup.

Phase 3b of EFFECTIVE_RATE_OPTIMIZER_DESIGN.md. Sweeps (config, channel)
cells on the IONOS hardware testbed and writes a JSON table mapping
(config_id, channel_condition) -> measured effective bps + sack rate stats.

The optimizer treats this as a Q-table lookup: given a current measured
(eff_bps, sack_rate) on the active config, look up what each other config
WOULD have delivered under similar channel conditions, and pick the
argmax (with hysteresis).

Why empirical? PHY rate is known analytically (arq_common.cc:1148-1198)
but SAC overhead is not — it depends on PTT delays, channel symmetry,
SACK_RSP wire time, retransmit batch sizing. Easier to MEASURE once on
IONOS than model. (Design doc §2.3.)

DESIGN POINTS
-------------
1. **Fixed config per cell**: `-s <cfg> --skip-turbo-reverse`, NO `-g`.
   We characterize THAT config in isolation; gearshift would muddy the
   signal by switching mid-run.
2. **Compression on**: `-F on` matches the production use case for text
   payloads (pg84.txt). Binary payloads would skew the sack_rate signal
   because PPMd is bypassed (entropy guard).
3. **SACK enabled**: `--enable-sack --enable-sack-v2` — the optimizer is
   downstream of Design A; it has to assume SACK is on the wire.
4. **Per-cell 3 runs**: variance estimate. We record min/max/mean.
5. **Lossy cells log actual loss**: IONOS channel points are nominal —
   real per-batch loss is harvested from [CMD-SACK] events.
6. **Skip-on-fail**: cells where mercury can't even connect get
   `failed=true, eff_bps_mean=0`. The optimizer treats those as
   "do not pick this config under this loss profile".
7. **Resumable**: `--resume` skips cells already present in the JSON.
   A partial run can be continued without losing prior cells.
8. **Sequential only**: butler holds one lease at a time. No parallelism.

The harvest comes from TWO sources:
* [OPT-WINDOW] log lines emitted by Phase 3a plumbing (arq.h:1392) —
  rolling effective bps + sack_rate every 10 batches.
* The TCP receiver byte count + per-cell run duration — fallback /
  ground truth if [OPT-WINDOW] is absent.

RUNTIME ESTIMATE
----------------
configs x channels x runs x (duration + ~25s setup overhead per run)
  full:  8 x 14 x 3 x (150 + 25) = ~17 hours
  quick: 8 x 14 x 1 x (300 + 25) =  ~12.6 hours (single-run; covers cfg=6 startup)
  dev:   sub-selection via --configs / --points

USAGE
-----
  python tools/sim/effective_rate_calibrate.py
      [--out PATH]            default mercury/effective_rate_table.json
      [--configs LIST]        comma-list, default 6,9,11,12,13,14,15,16
      [--points  LIST]        comma-list, default full channel set
      [--runs    N]           runs per cell, default 3
      [--duration S]          seconds per run, default 150
      [--quick]               1 run x 300s per cell (~12.6h; covers cfg=6 startup)
      [--full]                3 runs x 150s per cell (~17h, default)
      [--resume]              skip cells already in OUT
      [--payload PATH]        default pg84.txt
      [--settle-s N]          drop first N seconds from bps (default 30)

OUTPUT SCHEMA (effective_rate_table.json)
-----------------------------------------
{
  "schema_version": 2,                       # bump on any breaking change
  "calibration_date": "ISO8601 UTC",
  "mercury_head": "<git short sha>",
  "calibration_setup": {
    "compress":     false,                  # raw PHY prior; runtime converts units
    "sack":         true,                   # --enable-sack on
    "sack_v2":      true,                   # --enable-sack-v2 on
    "gearshift":    false,                  # fixed config, no -g
    "payload":      "pg84.txt",
    "duration_s":   150,
    "settle_s":     30,
    "runs_per_cell":3,
    "audio_dev":    "plughw:Audio"
  },
  "configs_tested":  [6, 9, 11, 12, 13, 14, 15, 16],
  "channels_tested": ["clean", "wgn30", ...],
  "channel_definitions": {                   # cite the channel commands
    "clean":  ["WGN:40", ...],
    ...
  },
  "table": {
    "<config_id>": {
      "<channel_name>": {
        # primary axes the optimizer reads:
        "eff_bps_mean":      <float>,        # mean across runs
        "eff_bps_min":       <float>,
        "eff_bps_max":       <float>,
        "eff_bps_sigma":     <float>,
        "sack_rate_mean":    <float>,        # 0.0-1.0; from [OPT-WINDOW] last sample
        "batch_count_mean":  <float>,        # batches per run
        "frame_loss_pct":    <float>,        # mean per-batch loss from [CMD-SACK]
        # provenance:
        "n_runs":            <int>,          # successful runs in mean
        "n_failed_runs":     <int>,          # connection/launch failures
        "failed":            <bool>,         # ALL runs failed
        "break_fired":       <bool>,         # any run logged BREAK
        # raw run series for re-aggregation:
        "runs": [
          {"eff_bps": ..., "sack_rate": ..., "batch_count": ...,
           "frame_loss_pct": ..., "rx_bytes": ..., "duration_s": ...,
           "connected": ..., "break_fired": ..., "error": ...,
           "timestamp": "..."},
          ...
        ]
      },
      ...
    },
    ...
  },
  "started":  "YYYYMMDD_HHMMSS",
  "finished": "YYYYMMDD_HHMMSS",
  "total_cells":   <int>,
  "total_runs":    <int>,
  "total_runtime_s": <float>,
  "total_rx_bytes":  <int>
}

SCHEMA STABILITY
----------------
Phase 3c reads:
  table[str(cfg_id)][channel_name]["eff_bps_mean"]
  table[str(cfg_id)][channel_name]["sack_rate_mean"]
  table[str(cfg_id)][channel_name]["frame_loss_pct"]
  table[str(cfg_id)][channel_name]["failed"]

Any change to those four keys is a BREAKING change and requires a
schema_version bump + Phase 3c reader update.
"""
import argparse
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time

if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Migrated into the mercury repo at mercury/tools/sim/ (was tools/ in the outer
# workspace). Anchor to the mercury repo root and the outer workspace root so the
# default payload (pg84.txt), the default output (mercury/effective_rate_table.json),
# and the mercury git-head lookup all keep resolving to the real locations.
MERCURY_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))   # mercury/tools/sim -> mercury
WORKSPACE    = os.path.dirname(MERCURY_ROOT)                  # mercury -> workspace root

# Reuse the harness vocabulary verbatim from sack_lossy_ab.py so a wiring
# change on the testbed flows through one place. We import lazily because
# sack_lossy_ab.py's top-level does no work, but is large.
sys.path.insert(0, SCRIPT_DIR)
import sack_lossy_ab as SLA  # noqa: E402

BUTLER       = SLA.BUTLER
PI_RSP_HOST  = SLA.PI_RSP_HOST
PI_CMD_HOST  = SLA.PI_CMD_HOST
PI_RSP_PORT  = SLA.PI_RSP_PORT
PI_CMD_PORT  = SLA.PI_CMD_PORT
PI_MERCURY   = SLA.PI_MERCURY
PI_AUDIO_DEV = SLA.PI_AUDIO_DEV
CMD_LOG      = SLA.CMD_LOG
RSP_LOG      = SLA.RSP_LOG
CHANNEL_POINTS = SLA.CHANNEL_POINTS

b_send  = SLA.b_send
b_lock  = SLA.b_lock
tcp_connect_retry = SLA.tcp_connect_retry
tcp_send_line     = SLA.tcp_send_line
tcp_recv_drain    = SLA.tcp_recv_drain
tcp_recv_until    = SLA.tcp_recv_until
receiver_thread   = SLA.receiver_thread
sender_thread     = SLA.sender_thread

SCHEMA_VERSION = 2

# Configs to sweep — WB only. Lower configs (<6) are too slow to be the
# optimizer's pick under any realistic channel; high configs dominate the
# decision space. CFG7/8/10 omitted by design (CFG6 covers low rates,
# CFG9 covers mid, ladder steps thereafter).
DEFAULT_CONFIGS = [6, 9, 11, 12, 13, 14, 15, 16]

# Channel sweep — as specified in the task. Use sack_lossy_ab.CHANNEL_POINTS
# keys verbatim so the strings match what's already documented.
DEFAULT_POINTS = [
    'clean',
    'wgn30', 'wgn28', 'wgn26', 'wgn24', 'wgn22', 'wgn20', 'wgn18', 'wgn16',
    'mpp22', 'mpp18', 'mpp16',
    'mpm18', 'mpm16',
]


def parse_opt_window_metrics(cmd_text):
    """Harvest [OPT-WINDOW] samples from Phase 3a plumbing.

    Format (arq.h:1392):
      [OPT-WINDOW] eff_bps=<f> sack_rate=<f> window_n=<i> cfg=<i>

    Returns a list of {eff_bps, sack_rate, window_n, cfg} dicts in order.
    """
    samples = []
    # New format adds frame_ok/partial_loss/fail_rate. Keep the legacy regex
    # fallback so archived logs remain harvestable.
    new_re = re.compile(
        r'\[OPT-WINDOW\]\s+eff_bps=([\d.]+)\s+sack_rate=([\d.]+)\s+'
        r'frame_ok=([\d.]+)\s+partial_loss=([\d.]+)\s+'
        r'fail_rate=([\d.]+)\s+window_n=(\d+)\s+cfg=(\d+)')
    for mo in new_re.finditer(cmd_text):
        samples.append({
            'eff_bps': float(mo.group(1)), 'sack_rate': float(mo.group(2)),
            'frame_ok': float(mo.group(3)), 'partial_loss': float(mo.group(4)),
            'fail_rate': float(mo.group(5)), 'window_n': int(mo.group(6)),
            'cfg': int(mo.group(7)),
        })
    if not samples:
        for mo in re.finditer(
                r'\[OPT-WINDOW\]\s+eff_bps=([\d.]+)\s+sack_rate=([\d.]+)\s+'
                r'window_n=(\d+)\s+cfg=(\d+)', cmd_text):
            samples.append({
                'eff_bps': float(mo.group(1)), 'sack_rate': float(mo.group(2)),
                'window_n': int(mo.group(3)), 'cfg': int(mo.group(4)),
            })
    return samples



def parse_opt_batch_records(cmd_text):
    """Parse immutable Gearshift-v2 primitive batch records.

    Returns one dict per [OPT-BATCH], with later [OPT-APP-COMMIT] credit
    joined by record id.  These are non-overlapping transaction records, unlike
    rolling [OPT-WINDOW] snapshots, so aggregate arithmetic is well defined.
    """
    batch_re = re.compile(
        r'\[OPT-BATCH\]\s+id=(\d+)\s+cfg=(\d+)\s+rate_valid=(\d+)\s+'
        r'outcome_valid=(\d+)\s+cycle_ms=(\d+)\s+app_bytes=(\d+)\s+'
        r'transport_bytes=(\d+)\s+sent=(\d+)\s+acked=(\d+)\s+'
        r'sack=(\d+)\s+failed=(\d+)\s+batch_size=(\d+)\s+'
        r'snr=([-+\d.]+)\s+snr_age=(\d+)\s+sel=([-+\d.]+)'
        r'(?:\s+lineage_valid=(\d+)\s+new_sent=(\d+)\s+repair_sent=(\d+)'
        r'\s+cfg_gen=(\d+)\s+unit_id=(\d+)\s+dir=([A-Za-z0-9_-]+))?')
    commit_re = re.compile(
        r'\[OPT-APP-COMMIT\]\s+id=(\d+)\s+cfg=(\d+)\s+app_bytes=(\d+)'
        r'(?:\s+unit_id=(\d+))?')
    commits = {}
    for m in commit_re.finditer(cmd_text):
        commits[int(m.group(1))] = commits.get(int(m.group(1)), 0) + int(m.group(3))
    out = []
    for m in batch_re.finditer(cmd_text):
        rid = int(m.group(1))
        out.append({
            'id': rid, 'cfg': int(m.group(2)), 'rate_valid': int(m.group(3)),
            'outcome_valid': int(m.group(4)), 'cycle_ms': int(m.group(5)),
            'app_bytes': int(m.group(6)) + commits.get(rid, 0),
            'transport_bytes': int(m.group(7)), 'sent': int(m.group(8)),
            'acked': int(m.group(9)), 'sack': int(m.group(10)),
            'failed': int(m.group(11)), 'batch_size': int(m.group(12)),
            'snr': float(m.group(13)), 'snr_age': int(m.group(14)),
            'selectivity': float(m.group(15)),
            # The suffix was added by Gearshift-v2 checkpoint 3.  Archived v2
            # primitive logs without it remain parseable; unknown lineage is
            # explicit rather than guessed from ACK/SACK outcome.
            'lineage_valid': int(m.group(16)) if m.group(16) is not None else 0,
            'new_sent': int(m.group(17)) if m.group(17) is not None else 0,
            'repair_sent': int(m.group(18)) if m.group(18) is not None else 0,
            'config_generation': int(m.group(19)) if m.group(19) is not None else None,
            'application_unit_id': int(m.group(20)) if m.group(20) is not None else None,
            'direction': m.group(21) if m.group(21) is not None else None,
        })
    return out

def filter_primitive_records_for_config(records, target_cfg):
    """Keep only records belonging to the fixed action being calibrated.

    Emergency BREAK is allowed to recover the live link, but records emitted
    after the modem has changed configuration are not counterfactual evidence
    for the requested fixed action.
    """
    if target_cfg is None:
        return list(records)
    target_cfg = int(target_cfg)
    return [r for r in records if int(r.get('cfg', -1)) == target_cfg]

def harvest_run(result, cmd_text, rsp_text):
    """Harvest one calibration run.

    Gearshift-v2 primitive [OPT-BATCH] records are authoritative when present.
    Legacy rolling-window/log parsers remain as backward-compatible fallback.
    """
    primitive_all = parse_opt_batch_records(cmd_text)
    # A fixed-config calibration cell must never absorb bytes from an emergency
    # demotion/recovery action.  BREAK is allowed to save the live link, but any
    # later CONFIG_*/ROBUST_* transactions describe a different action.  Keep
    # only primitive records emitted at the requested fixed configuration.
    target_cfg = result.get('config_id')
    primitive = filter_primitive_records_for_config(primitive_all, target_cfg)
    mech = SLA.parse_mercury_logs(cmd_text, rsp_text)
    if primitive:
        rate = [r for r in primitive if r['rate_valid'] and r['cycle_ms'] > 0]
        outcome = [r for r in primitive if r['outcome_valid']]
        cycle_ms = sum(r['cycle_ms'] for r in rate)
        app_bytes = sum(r['app_bytes'] for r in rate)
        transport_bytes = sum(r['transport_bytes'] for r in rate)
        # Application bytes are the objective when commit records exist.  A
        # fixed-mode calibration run with no app commit is a genuine 0-app-bps
        # outcome rather than missing evidence.  Transport bps is retained for
        # diagnosis/bootstrapping only.
        app_bps = (app_bytes * 8000.0 / cycle_ms) if cycle_ms > 0 else 0.0
        transport_bps = (transport_bytes * 8000.0 / cycle_ms) if cycle_ms > 0 else 0.0
        sacks = sum(r['sack'] for r in outcome)
        fails = sum(r['failed'] for r in outcome)
        sent = sum(r['sent'] for r in outcome)
        acked = sum(min(r['acked'], r['sent']) for r in outcome)
        partial = [r for r in outcome if r['sack'] and r['sent'] > 0]
        partial_sent = sum(r['sent'] for r in partial)
        partial_missing = sum(r['sent'] - min(r['acked'], r['sent']) for r in partial)
        sack_rate = sacks / len(outcome) if outcome else 0.0
        failed_rate = fails / len(outcome) if outcome else 0.0
        frame_success = acked / sent if sent else 1.0
        frame_loss_pct = 100.0 * partial_missing / partial_sent if partial_sent else 0.0
        sels = [r['selectivity'] for r in outcome if r['selectivity'] >= 0.0]
        snrs = [r['snr'] for r in outcome if r['snr'] > -90.0 and r['snr_age'] <= 4]
        batch_sizes = [r['batch_size'] for r in outcome if r['batch_size'] > 0]
        lineage = [r for r in outcome if r.get('lineage_valid')]
        lineage_sent = sum(r['new_sent'] + r['repair_sent'] for r in lineage)
        repair_frames = sum(r['repair_sent'] for r in lineage)
        return {
            'eff_bps': round(app_bps, 1),
            'transport_bps': round(transport_bps, 1),
            'eff_bps_source': 'opt_batch_application',
            'sack_rate': round(sack_rate, 4),
            'failed_batch_rate': round(failed_rate, 4),
            'frame_success': round(frame_success, 4),
            'batch_count': len(outcome),
            'batch_size': round(sum(batch_sizes)/len(batch_sizes), 3) if batch_sizes else None,
            'frame_loss_pct': round(frame_loss_pct, 2),
            'selectivity': round(sum(sels)/len(sels), 6) if sels else None,
            'snr_db': round(sum(snrs)/len(snrs), 3) if snrs else None,
            'break_fired': bool(re.search(r'\[TX-BREAK\] Sending BREAK pattern', cmd_text)) or
                           bool(re.search(r'emergency.*nack.*threshold', cmd_text, re.I)),
            'primitive_records': len(primitive),
            'primitive_records_total': len(primitive_all),
            'lineage_records': len(lineage),
            'repair_frame_rate': round(repair_frames / lineage_sent, 6) if lineage_sent else None,
            'mech_summary': {
                'cmd_nSent_data': mech.get('cmd_nSent_data'),
                'cmd_nReSent_data': mech.get('cmd_nReSent_data'),
                'cmd_nLost_data': mech.get('cmd_nLost_data'),
                'n_cmd_sack_events': mech.get('n_cmd_sack_events'),
                'cmd_retx_frames_total': mech.get('cmd_retx_frames_total'),
                'gearshift_lines': mech.get('gearshift_lines'),
            },
        }

    # Legacy fallback for archived logs without primitive records.
    opt_samples = parse_opt_window_metrics(cmd_text)
    eff_samples = [s['eff_bps'] for s in opt_samples[1:]] if len(opt_samples)>1 else [s['eff_bps'] for s in opt_samples]
    if eff_samples:
        eff_bps = sum(eff_samples)/len(eff_samples); eff_source='opt_window_legacy'
    elif result.get('measured_s',0)>0:
        eff_bps=(result.get('rx_bytes_measured',0)*8.0)/result['measured_s']; eff_source='tcp_bytes'
    else:
        eff_bps=0.0; eff_source='none'
    sr=[s['sack_rate'] for s in opt_samples[1:]] if len(opt_samples)>1 else [s['sack_rate'] for s in opt_samples]
    sack_rate=sum(sr)/len(sr) if sr else 0.0
    nbatch=mech.get('n_batch_tx_start') or 0
    if not sr and nbatch>0: sack_rate=(mech.get('n_cmd_sack_events') or 0)/nbatch
    pls=[s['partial_loss'] for s in opt_samples[1:] if 'partial_loss' in s] if len(opt_samples)>1 else [s['partial_loss'] for s in opt_samples if 'partial_loss' in s]
    frame_loss_pct=(sum(pls)/len(pls))*100.0 if pls else ((mech.get('sack_per_batch_loss_mean') or 0.0)*100.0)
    return {
        'eff_bps':round(eff_bps,1),'eff_bps_source':eff_source,
        'sack_rate':round(sack_rate,4),'batch_count':nbatch,'batch_size':None,
        'frame_loss_pct':round(frame_loss_pct,2),
        'failed_batch_rate':0.0,'frame_success':1.0,
        'selectivity':None,'snr_db':None,
        'break_fired':bool(re.search(r'\[TX-BREAK\] Sending BREAK pattern',cmd_text)) or bool(re.search(r'emergency.*nack.*threshold',cmd_text,re.I)),
        'primitive_records':0,'mech_summary':{'cmd_nSent_data':mech.get('cmd_nSent_data'),'cmd_nReSent_data':mech.get('cmd_nReSent_data'),'cmd_nLost_data':mech.get('cmd_nLost_data'),'n_cmd_sack_events':mech.get('n_cmd_sack_events'),'cmd_retx_frames_total':mech.get('cmd_retx_frames_total'),'gearshift_lines':mech.get('gearshift_lines')},
    }



def build_cell_run(raw, harvest, run_idx):
    """Join one harness result with harvested Gearshift-v2 metrics.

    Keep every v2 primitive field all the way into table[config][channel][runs];
    dropping a field here would make aggregate_cell silently fall back to neutral
    defaults even though the modem emitted the measurement.
    """
    return {
        'eff_bps': harvest.get('eff_bps', 0.0),
        'transport_bps': harvest.get('transport_bps', 0.0),
        'eff_bps_source': harvest.get('eff_bps_source', 'none'),
        'sack_rate': harvest.get('sack_rate', 0.0),
        'failed_batch_rate': harvest.get('failed_batch_rate', 0.0),
        'frame_success': harvest.get('frame_success', 1.0),
        'batch_count': harvest.get('batch_count', 0),
        'batch_size': harvest.get('batch_size'),
        'frame_loss_pct': harvest.get('frame_loss_pct', 0.0),
        'selectivity': harvest.get('selectivity'),
        'snr_db': harvest.get('snr_db'),
        'primitive_records': harvest.get('primitive_records', 0),
        'primitive_records_total': harvest.get('primitive_records_total',
                                                harvest.get('primitive_records', 0)),
        'lineage_records': harvest.get('lineage_records', 0),
        'repair_frame_rate': harvest.get('repair_frame_rate'),
        'opt_window_samples': harvest.get('opt_window_samples', 0),
        'break_fired': harvest.get('break_fired', False),
        'rx_bytes': raw.get('rx_bytes', 0),
        'rx_bytes_measured': raw.get('rx_bytes_measured', 0),
        'tx_bytes': raw.get('tx_bytes', 0),
        'duration_s': raw.get('duration_s', 0),
        'measured_s': raw.get('measured_s', 0),
        'connected': raw.get('connected', False),
        'error': raw.get('error'),
        'timestamp': raw.get('timestamp'),
        'cmd_log': raw.get('cmd_log'),
        'rsp_log': raw.get('rsp_log'),
        'mech_summary': harvest.get('mech_summary', {}),
        'run_idx': run_idx,
    }

def aggregate_cell(run_dicts):
    """Aggregate a cell without survivor bias.

    Every connected, instrument-valid run is a performance outcome, including
    valid zero-delivery runs.  Infrastructure/launch failures are counted
    separately and do not manufacture zero RF performance.
    """
    valid=[r for r in run_dicts if r.get('connected') and not r.get('error')]
    invalid=[r for r in run_dicts if not r.get('connected') or r.get('error')]
    def stat(values):
        if not values:return (0.0,0.0,0.0,0.0)
        n=len(values);m=sum(values)/n
        sig=(sum((v-m)**2 for v in values)/n)**0.5 if n>1 else 0.0
        return (round(m,1),round(min(values),1),round(max(values),1),round(sig,1))
    eff=[float(r.get('eff_bps',0.0)) for r in valid]
    emn,emin,emax,esig=stat(eff)
    def mean(field,default=0.0):
        vals=[r.get(field) for r in valid if r.get(field) is not None]
        return round(sum(vals)/len(vals),6) if vals else default
    return {
        'eff_bps_mean':emn,'eff_bps_min':emin,'eff_bps_max':emax,'eff_bps_sigma':esig,
        'sack_rate_mean':mean('sack_rate'),'batch_count_mean':mean('batch_count'),
        'batch_size_mean':mean('batch_size',None),
        'frame_loss_pct':mean('frame_loss_pct'),'failed_batch_rate_mean':mean('failed_batch_rate'),
        'frame_success_mean':mean('frame_success',1.0),'selectivity_mean':mean('selectivity',None),
        'repair_frame_rate_mean':mean('repair_frame_rate',None),
        'snr_db':mean('snr_db',None),'n_runs':len(valid),'n_zero_delivery_runs':sum(1 for r in valid if float(r.get('eff_bps',0.0))<=0.0),
        'n_invalid_runs':len(invalid),'n_failed_runs':len(invalid),
        'failed':(len(valid)==0 and len(run_dicts)>0),
        'break_fired':any(r.get('break_fired') for r in valid),
        'break_run_rate':round(sum(1 for r in valid if r.get('break_fired')) / len(valid), 6) if valid else 0.0,
        'outcome_semantics':'v2_all_valid_runs_including_zero',
        'runs':run_dicts,
    }


def get_mercury_head():
    """Best-effort git short SHA of the mercury repo (this script now lives
    inside it at tools/sim/)."""
    mercury_dir = MERCURY_ROOT
    try:
        out = subprocess.check_output(
            ['git', '-C', mercury_dir, 'rev-parse', '--short', 'HEAD'],
            stderr=subprocess.DEVNULL, timeout=10)
        return out.decode().strip()
    except Exception:
        return 'unknown'


def get_config_signature():
    """Version the calibration against config/protocol geometry that changes
    the action set or SACK semantics. Missing/old signatures remain readable
    but Gearshift-v2 deliberately weakens those priors."""
    path = os.path.join(MERCURY_ROOT, 'include', 'common', 'common_defines.h')
    try:
        text = open(path, 'r', encoding='utf-8', errors='ignore').read()
        def val(name, default):
            m = re.search(r'^\s*#define\s+' + re.escape(name) + r'\s+([^\s/]+)', text, re.M)
            return m.group(1) if m else str(default)
        return 'gs2-cfg%s-wb%s-nb%s-sack%s-bitmap%s' % (
            val('NUMBER_OF_CONFIGS', 0), val('WB_CONFIG_MAX', 0).replace('CONFIG_', ''),
            val('NB_CONFIG_MAX', 0).replace('CONFIG_', ''), val('MFSK_ACK_SACK_ENABLED', 0),
            val('MFSK_SACK_BITMAP_BITS', 0))
    except Exception:
        return 'unknown'


def load_existing(path):
    """Read a prior calibration JSON for --resume. Returns None if absent."""
    if not os.path.exists(path):
        return None
    try:
        with open(path, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f'WARN: cannot read {path} for resume: {e}; ignoring')
        return None



def resume_metadata_compatible(prior, mercury_head, config_signature):
    """Return (ok, reason) for a safe --resume.

    A table may only be resumed when every retained cell was produced by the
    same source/config geometry.  Updating root metadata while silently keeping
    old cells would make stale measurements look freshly calibrated.
    """
    if not prior:
        return True, ''
    old_sig = prior.get('config_signature')
    old_head = prior.get('mercury_head')
    if not old_sig:
        return False, 'prior table has no config_signature'
    if old_sig != config_signature:
        return False, f'config_signature mismatch prior={old_sig} current={config_signature}'
    if not old_head or old_head in ('unknown', 'nogit'):
        return False, 'prior table has no trustworthy mercury_head'
    if mercury_head not in ('unknown', 'nogit') and old_head != mercury_head:
        return False, f'mercury_head mismatch prior={old_head} current={mercury_head}'
    return True, ''

def cell_already_done(prior, cfg_id, channel, required_runs, retry_failed=False,
                      is_nb=False):
    """True if the prior JSON has a complete cell for (cfg_id, channel).

    With retry_failed=True, cells where every run failed (failed=true) are
    treated as NOT done — they get re-scheduled. This is the recovery path
    after a Pi-audio-state-drift incident contaminated a portion of the
    sweep. See memory/pi_audio_state_drift.md.

    is_nb selects "table_nb" vs "table" section. NB-mode resume reads only
    NB cells; WB-mode resume reads only WB cells.
    """
    if not prior:
        return False
    section_key = 'table_nb' if is_nb else 'table'
    t = prior.get(section_key, {})
    cell = t.get(str(cfg_id), {}).get(channel)
    if not cell:
        return False
    if retry_failed and cell.get('failed', False):
        return False
    return len(cell.get('runs', [])) >= required_runs


def reboot_pis_via_butler(reason=''):
    """Reboot both Pis, wait for them to come back online + ALSA settle.

    Used to flush the Pi audio-state drift that accumulates over long runs
    (see memory/pi_audio_state_drift.md). Costs ~70-90s total.
    """
    import socket as _sock
    def _send(s, msg, timeout=20):
        s.settimeout(timeout)
        s.sendall((msg + '\n').encode())
        buf = b''
        try:
            while True:
                chunk = s.recv(8192)
                if not chunk:
                    break
                buf += chunk
                if (buf.endswith(b'\n') or buf.endswith(b'OK\n')
                        or b'FAIL' in buf):
                    break
        except _sock.timeout:
            pass
        return buf.decode(errors='replace')

    print(f'\n[REBOOT] {reason} — rebooting both Pis...')
    s = _sock.socket(); s.connect(('localhost', 7700))
    resp = _send(s, 'LOCK reboot_periodic 120')
    if 'OK' not in resp:
        print(f'[REBOOT] LOCK failed: {resp.strip()}; skipping reboot')
        s.close()
        return False
    lid = resp.split()[1].strip()
    try:
        for rpi in ('rpi1', 'rpi2'):
            # Detached background reboot — SSH connection drops immediately
            _send(s, f'SSH {lid} {rpi} killall -9 mercury 2>/dev/null; '
                     f'echo killed', timeout=10)
            _send(s, f'SSH {lid} {rpi} nohup sudo reboot >/dev/null 2>&1 &',
                  timeout=10)
        _send(s, f'UNLOCK {lid}')
    finally:
        s.close()

    # Wait for both Pis to be SSH-responsive again.
    print('[REBOOT] Waiting for Pis to come back...')
    time.sleep(30)
    for attempt in range(30):
        s = _sock.socket(); s.connect(('localhost', 7700))
        resp = _send(s, 'LOCK reboot_wait 60')
        if 'OK' not in resp:
            s.close(); time.sleep(10); continue
        lid = resp.split()[1].strip()
        r1 = _send(s, f'SSH {lid} rpi1 echo OK_rpi1', timeout=10)
        r2 = _send(s, f'SSH {lid} rpi2 echo OK_rpi2', timeout=10)
        _send(s, f'UNLOCK {lid}'); s.close()
        if 'OK_rpi1' in r1 and 'OK_rpi2' in r2:
            print(f'[REBOOT] Both Pis back online (after {30+attempt*10}s). '
                  f'Sleeping 30s for ALSA settle...')
            time.sleep(30)
            return True
        time.sleep(10)
    print('[REBOOT] WARNING: Pis did not come back within timeout — continuing anyway')
    return False


def _hung_cell_watchdog(lid, main_bs, main_done, deadline, log_prefix):
    # Per-cell deadline guard. SLA.run_one has no wall-clock timeout; when
    # mercury fails to CONNECT it can block a TCP recv indefinitely, the
    # main thread never reaches the UNLOCK in the caller's finally, and the
    # butler lease persists via the EXTEND chain for ~30 min while the
    # calibrator skips every remaining cell with BUSY. The watchdog opens
    # a FRESH butler socket so we can act while main_bs is wedged in recv:
    # kill mercury on both Pis so SLA.run_one's recv unblocks, UNLOCK the
    # lease (butler accepts UNLOCK from any client with the right lid),
    # then shutdown main_bs so any pending butler recv in SLA.run_one
    # returns immediately and propagates an exception up.
    while True:
        if main_done.wait(timeout=5):
            return
        if time.time() >= deadline:
            break
    print(f'{log_prefix} watchdog: cell exceeded deadline; '
          'killing mercury + forcing UNLOCK', flush=True)
    try:
        wd_bs = socket.socket()
        wd_bs.settimeout(15)
        wd_bs.connect(BUTLER)
        for rpi in ('rpi1', 'rpi2'):
            try:
                b_send(wd_bs, f'SSH {lid} {rpi} killall -9 mercury 2>/dev/null; '
                              f'echo killed', t=10)
            except Exception as e:
                print(f'{log_prefix} watchdog kill {rpi}: {e}', flush=True)
        try:
            b_send(wd_bs, f'UNLOCK {lid}', t=5)
        except Exception:
            pass
        try:
            wd_bs.close()
        except Exception:
            pass
    except Exception as e:
        print(f'{log_prefix} watchdog wd_bs failed: {e}', flush=True)
    try:
        main_bs.shutdown(socket.SHUT_RDWR)
    except Exception:
        pass
    try:
        main_bs.close()
    except Exception:
        pass


def run_one_calibration_cell(bs, lid, cfg_id, channel, channel_cmds,
                             duration_s, settle_s, payload, out_dir,
                             run_idx, total_runs, is_nb=False):
    """One run for (cfg_id, channel, run_idx). Returns a harvested dict.

    Built on top of sack_lossy_ab.run_one for the launch/connect/measure
    machinery, then we re-parse the run's logs with harvest_run() to extract
    the [OPT-WINDOW] series the calibration table needs.

    Calibration always: gearshift OFF, compression OFF, SACK v2 ON.

    Compression is OFF because the Q-table needs RAW PHY rate per
    (cfg, channel) — that's the channel-bound quantity the optimizer ranks
    configs by. Compression ratio depends on payload entropy and ppmd/zstd
    warmup; including it makes the table a moving target. Production rate
    (with compression) is measured separately in tools/phase4_benchmark.py.
    """
    cfg_label = f'CFG{cfg_id}'
    # 2026-05-29: per-run wall-clock deadline (the watchdog's whole purpose).
    # SLA.run_one launches mercury on the REMOTE Pis via butler SSH — there is
    # no LOCAL subprocess to reap; the wedge is local socket blocking (a TCP
    # recv on mercury's control port that never reaches CONNECTED, or a butler
    # recv) plus the lease sitting alive on the EXTEND chain. The deadline must
    # be generous enough that a legitimately-slow cell (slow connect at deep
    # SNR + 600s measurement window) NEVER false-fires, yet bound a true wedge.
    #
    # Budget = expected run wall-clock + adaptive margin. Expected run time is
    # the measurement window (settle_s + duration_s) plus ~120s of fixed
    # setup/teardown overhead (kill+truncate logs, AUDIO_SETUP, mercury launch,
    # two tcp_connect_retry rounds up to ~40s each, DOWNLOAD ×2 at t=90, log
    # parse). The adaptive margin is the task's max(window*1.5, window+300),
    # rewritten as window + max(window*0.5, 300): at a 240s window that is
    # 270 + max(135,300)=300 → +120 setup = 690s; at a 600s window it is
    # 630 + max(315,300)=315 → +120 = 1065s. Both sit comfortably above the
    # respective worst-case run wall-clocks (~650s / ~1010s) so slow cells
    # are never killed, while a genuine hang is bounded to ~12-18 min instead
    # of the ~30 min orphan-lease window that truncated the v10 sweep at 36/99.
    run_window_s = settle_s + duration_s
    setup_teardown_s = 120
    deadline_budget_s = run_window_s + max(run_window_s * 0.5, 300) + setup_teardown_s
    deadline = time.time() + deadline_budget_s
    main_done = threading.Event()
    log_prefix = f'  [CAL cfg={cfg_id} ch={channel} r{run_idx}]'
    wd = threading.Thread(
        target=_hung_cell_watchdog,
        args=(lid, bs, main_done, deadline, log_prefix),
        daemon=True)
    wd.start()
    timed_out_flag = False
    raw = None
    try:
        raw = SLA.run_one(
            bs, lid, channel, channel_cmds, cfg_label, cfg_id,
            is_nb=is_nb, mode='sackv2', run_idx=run_idx,
            duration_s=duration_s, payload=payload, out_dir=out_dir,
            settle_s=settle_s, sack_rx_trace=False,
            gearshift=False,        # fixed config — this is the whole point
            compress='off',         # raw PHY for Q-table decision-anchoring
            max_config=None,
            optimizer_disabled=True,  # Phase 3c kill switch: calibration measures
                                      # each fixed (cfg, channel) cell without the
                                      # optimizer trying to switch configs mid-run.
            runtime_env={'MERCURY_GS2_PRIMITIVE_TRACE': '1'},
        )
    except Exception as e:
        if time.time() >= deadline:
            timed_out_flag = True
            print(f'{log_prefix} hung beyond deadline '
                  f'({deadline_budget_s:.0f}s); watchdog cleaned up. '
                  f'Marking cell failed. Underlying exception: {type(e).__name__}: {e}',
                  flush=True)
        else:
            main_done.set()
            wd.join(timeout=2)
            raise
    finally:
        main_done.set()
        wd.join(timeout=5)

    if raw is None or timed_out_flag:
        # Synthesize a failed-cell raw dict so harvest still produces a row.
        # aggregate_cell will see failed=True and the next resume will pick
        # this cell up with --retry-failed.
        raw = {
            'point': channel, 'channel_cmds': channel_cmds,
            'config': cfg_label, 'config_id': cfg_id, 'is_nb': is_nb,
            'mode': 'sackv2', 'sack': True, 'run': run_idx,
            'duration_s': 0, 'bps': 0, 'rx_bytes': 0, 'tx_bytes': 0,
            'rx_bytes_at_settle': 0, 'rx_bytes_measured': 0,
            'settle_s': settle_s, 'measured_s': 0,
            'connected': False,
            'error': 'timeout_hung',
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'cmd_log': None, 'rsp_log': None,
        }

    # Re-load + parse the logs for OPT-WINDOW harvest.
    cmd_text = rsp_text = ''
    if raw.get('cmd_log') and os.path.exists(raw['cmd_log']):
        with open(raw['cmd_log'], 'r', encoding='utf-8', errors='replace') as f:
            cmd_text = f.read()
    if raw.get('rsp_log') and os.path.exists(raw['rsp_log']):
        with open(raw['rsp_log'], 'r', encoding='utf-8', errors='replace') as f:
            rsp_text = f.read()

    harvest = harvest_run(raw, cmd_text, rsp_text)

    return build_cell_run(raw, harvest, run_idx)


def main():
    ap = argparse.ArgumentParser(
        description='Sweep (config, channel) on IONOS testbed to produce '
                    'the effective-rate Q-table for Mercury\'s Phase 3c '
                    'optimizer.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Examples:\n'
            '  # Full calibration sweep (~17 hours):\n'
            '  python tools/sim/effective_rate_calibrate.py --full\n\n'
            '  # Quick dev sweep (1 run/cell x 60s, ~2.6h):\n'
            '  python tools/sim/effective_rate_calibrate.py --quick\n\n'
            '  # Resume an interrupted full sweep:\n'
            '  python tools/sim/effective_rate_calibrate.py --full --resume\n\n'
            '  # Spot-check a single config on one channel:\n'
            '  python tools/sim/effective_rate_calibrate.py '
            '--configs 15 --points wgn22 --runs 3 --duration 90\n'
        ))
    ap.add_argument('--out', default=os.path.join(MERCURY_ROOT,
                                                  'effective_rate_table.json'),
                    help='output JSON (default <mercury>/effective_rate_table.json)')
    ap.add_argument('--configs', default=None,
                    help=f'comma-separated config IDs '
                         f'(default {",".join(str(c) for c in DEFAULT_CONFIGS)})')
    ap.add_argument('--points', default=None,
                    help='comma-separated channel points (default full sweep)')
    ap.add_argument('--runs', type=int, default=None,
                    help='runs per cell (default 3; --quick overrides to 1)')
    ap.add_argument('--duration', type=int, default=None,
                    help='seconds per run (default 150; --quick overrides to 300)')
    ap.add_argument('--settle-s', type=int, default=30,
                    help='drop first N seconds from bps measurement (default 30)')
    ap.add_argument('--payload', default=os.path.join(WORKSPACE, 'pg84.txt'),
                    help='TX payload file (default pg84.txt)')
    ap.add_argument('--quick', action='store_true',
                    help='dev mode: 1 run/cell, 300s/run (~12.6h total; '
                         '60s was the historical default but undershoots cfg=6 '
                         'startup amortization — see fact-doc note 2026-05-23)')
    ap.add_argument('--full', action='store_true',
                    help='production: 3 runs/cell, 150s/run (~17h total) — default')
    ap.add_argument('--nb', action='store_true',
                    help='narrowband mode: calibrate NB configs instead of WB. '
                         'Output is written to "table_nb" section of the JSON '
                         '(parallel to the existing WB "table"). NB caps at '
                         'CONFIG_14 — passing higher --configs values is a '
                         'no-op at the PHY layer. Use NB-appropriate channels '
                         '(--points wgn8,wgn4,wgn0,wgn-5,... down to mercury\\\'s '
                         'NB cliff, typically ~-5 dB SNR).')
    ap.add_argument('--resume', action='store_true',
                    help='skip cells already present in --out (no re-run)')
    ap.add_argument('--retry-failed', action='store_true',
                    help='with --resume, RE-RUN cells where all runs failed '
                         '(failed=true). Use after a Pi-state-drift incident '
                         'to retry the cells the drift killed.')
    ap.add_argument('--reboot-every-h', type=float, default=0.0,
                    help='reboot both Pis every N hours to avoid the '
                         'audio-state-drift bug (0 = disable). Recommend 2.0 '
                         'for runs > ~3 hours.')
    ap.add_argument('--dry-run', action='store_true',
                    help='print the schedule and exit without touching hardware')
    args = ap.parse_args()

    if args.quick and args.full:
        print('ERROR: --quick and --full are mutually exclusive', file=sys.stderr)
        sys.exit(2)
    if args.quick:
        runs = args.runs if args.runs is not None else 1
        duration = args.duration if args.duration is not None else 300
        mode_label = 'quick'
    else:
        # Default = full (also if --full set explicitly)
        runs = args.runs if args.runs is not None else 3
        duration = args.duration if args.duration is not None else 150
        mode_label = 'full' if args.full else 'default'

    # Parse config / point lists.
    if args.configs:
        try:
            configs = [int(c.strip()) for c in args.configs.split(',') if c.strip()]
        except ValueError:
            print(f'ERROR: --configs must be comma-list of ints, got {args.configs!r}',
                  file=sys.stderr)
            sys.exit(2)
    else:
        configs = list(DEFAULT_CONFIGS)
    for c in configs:
        if not (0 <= c <= 17 or c in (100, 101, 102)):
            print(f'ERROR: config {c} is not a runnable calibrated config '
                  f'(allowed: 0..17,100..102)', file=sys.stderr)
            sys.exit(2)

    if args.points:
        points = [p.strip() for p in args.points.split(',') if p.strip()]
    else:
        points = list(DEFAULT_POINTS)
    for p in points:
        if p not in CHANNEL_POINTS:
            print(f'ERROR: channel point {p!r} not in CHANNEL_POINTS. '
                  f'Valid: {",".join(sorted(CHANNEL_POINTS))}',
                  file=sys.stderr)
            sys.exit(2)

    # Load resume state.
    prior = load_existing(args.out) if args.resume else None
    if args.resume and prior:
        print(f'[CAL] --resume: loaded prior {args.out} '
              f'(table has {len(prior.get("table", {}))} configs)')

    # Build schedule.
    schedule = []
    for cfg_id in configs:
        for channel in points:
            if args.resume and cell_already_done(prior, cfg_id, channel,
                                                runs, args.retry_failed,
                                                is_nb=args.nb):
                continue
            schedule.append((cfg_id, channel))
    total_cells = len(schedule)
    est_per_run_s = duration + 25  # ~25s overhead (mercury launch/connect/teardown)
    est_total_s   = total_cells * runs * est_per_run_s

    # Resolve payload.
    if not os.path.exists(args.payload):
        print(f'ERROR: payload {args.payload!r} not found', file=sys.stderr)
        sys.exit(2)
    payload = open(args.payload, 'rb').read()

    mercury_head = get_mercury_head()
    config_signature = get_config_signature()
    if prior and args.resume:
        compatible, why = resume_metadata_compatible(prior, mercury_head, config_signature)
        if not compatible:
            print(f'ERROR: refusing unsafe --resume: {why}. Start a new output table '
                  f'or rerun the old build instead of mixing calibration generations.',
                  file=sys.stderr)
            return 2
    print(f'[CAL] mode={mode_label} configs={configs} points={points}')
    print(f'[CAL] runs/cell={runs} duration/run={duration}s settle={args.settle_s}s')
    print(f'[CAL] payload={args.payload} ({len(payload)} bytes)')
    print(f'[CAL] mercury_head={mercury_head}')
    print(f'[CAL] config_signature={config_signature}')
    print(f'[CAL] out={args.out}')
    print(f'[CAL] schedule: {total_cells} cells x {runs} runs '
          f'= {total_cells * runs} runs')
    print(f'[CAL] est. runtime: {est_total_s/3600:.1f}h '
          f'({est_total_s/60:.0f} min)')

    if args.dry_run:
        print('[CAL] --dry-run: schedule preview:')
        for i, (cfg, ch) in enumerate(schedule, 1):
            print(f'  {i:3d}. CFG{cfg:<2d} ch={ch}')
        return 0

    # Initialize output document.
    if prior and args.resume:
        out = prior
        # Resume compatibility was verified above.  Root metadata can therefore
        # be refreshed without relabelling old cells from a different build.
        out['mercury_head'] = mercury_head
        out['config_signature'] = config_signature
        out['schema_version'] = SCHEMA_VERSION
        out.setdefault('calibration_setup', {})
        out.setdefault('configs_tested', [])
        for c in configs:
            if c not in out['configs_tested']:
                out['configs_tested'].append(c)
        out.setdefault('channels_tested', [])
        for p in points:
            if p not in out['channels_tested']:
                out['channels_tested'].append(p)
        out.setdefault('channel_definitions', {})
        out['channel_definitions'].update({p: CHANNEL_POINTS[p] for p in points})
        # Pick the section key based on bandwidth mode. WB → "table", NB → "table_nb".
        # Both sections coexist in the same JSON; mercury's rate_optimizer.cc
        # parses both at load() and routes lookups based on narrowband_enabled.
        table_key = 'table_nb' if args.nb else 'table'
        out.setdefault(table_key, {})
    else:
        table_key = 'table_nb' if args.nb else 'table'
        out = {
            'schema_version':    SCHEMA_VERSION,
            'calibration_date':  time.strftime('%Y-%m-%dT%H:%M:%SZ',
                                               time.gmtime()),
            'mercury_head':      mercury_head,
            'config_signature': config_signature,
            'calibration_setup': {
                'compress':      False,
                'sack':          True,
                'sack_v2':       True,
                'gearshift':     False,
                'narrowband':    bool(args.nb),
                'payload':       os.path.basename(args.payload),
                'duration_s':    duration,
                'settle_s':      args.settle_s,
                'runs_per_cell': runs,
                'audio_dev':     PI_AUDIO_DEV,
            },
            'configs_tested':       list(configs),
            'channels_tested':      list(points),
            'channel_definitions':  {p: CHANNEL_POINTS[p] for p in points},
            table_key:              {},
            'started':              time.strftime('%Y%m%d_%H%M%S'),
            'total_cells':          0,
            'total_runs':           0,
            'total_runtime_s':      0,
            'total_rx_bytes':       0,
        }

    out_dir = os.path.splitext(args.out)[0] + '_logs'
    os.makedirs(out_dir, exist_ok=True)

    def flush():
        # Update aggregates before writing. Sum cells across BOTH "table"
        # and "table_nb" so totals reflect the full document, not just the
        # currently-active section.
        nruns = 0
        nbytes = 0
        ncells = 0
        for sec in ('table', 'table_nb'):
            for cfg_id_str, ch_map in out.get(sec, {}).items():
                for ch_name, cell in ch_map.items():
                    if cell.get('runs'):
                        ncells += 1
                        nruns += len(cell['runs'])
                        nbytes += sum(int(r.get('rx_bytes', 0)) for r in cell['runs'])
        out['total_cells'] = ncells
        out['total_runs']  = nruns
        out['total_rx_bytes'] = nbytes
        os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
        tmp = args.out + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(out, f, indent=2)
        # atomic-ish replace so a crash mid-write can't corrupt the table
        os.replace(tmp, args.out)

    wall_start = time.time()
    last_reboot_t = wall_start  # treat the start as "just rebooted"
    reboot_every_s = args.reboot_every_h * 3600.0 if args.reboot_every_h > 0 else 0
    for cell_idx, (cfg_id, channel) in enumerate(schedule, 1):
        channel_cmds = CHANNEL_POINTS[channel]
        cfg_key = str(cfg_id)
        # Periodic Pi reboot to flush audio-state drift (see
        # memory/pi_audio_state_drift.md). Skip the FIRST cell since the
        # Pis are presumed already fresh at sweep start.
        if reboot_every_s > 0 and cell_idx > 1:
            since_reboot = time.time() - last_reboot_t
            if since_reboot >= reboot_every_s:
                hrs = since_reboot / 3600.0
                reboot_pis_via_butler(
                    reason=f'periodic ({hrs:.1f}h since last) before cell '
                           f'{cell_idx}/{total_cells}')
                last_reboot_t = time.time()
        print(f'\n{"="*72}\n'
              f'  [CAL] cell {cell_idx}/{total_cells}: cfg={cfg_id} ch={channel}\n'
              f'  channel = {channel_cmds}\n'
              f'{"="*72}')

        # One butler lease per cell — channel programmed once, runs interleave
        # under it. Matches sack_lossy_ab.py's per-point lease pattern.
        try:
            bs, lid = b_lock(f'effrate_cal_cfg{cfg_id}_{channel}', 600)
        except Exception as e:
            print(f'  [CAL] butler LOCK failed: {e}; skipping cell')
            continue

        cell_runs = []
        try:
            # Clean slate + program channel ONCE.
            for rpi in ('rpi1', 'rpi2'):
                b_send(bs, f'SSH {lid} {rpi} killall -9 mercury 2>/dev/null; '
                           f'echo done')
            for c in channel_cmds:
                r = b_send(bs, f'IONOS {lid} {c}', t=12)
                print(f'  IONOS {c} -> {r[:60]}')
                time.sleep(0.2)

            for run_idx in range(1, runs + 1):
                # Keep lease fresh before each run.
                b_send(bs, f'EXTEND {lid} 600')
                t_run = time.time()
                run = run_one_calibration_cell(
                    bs, lid, cfg_id, channel, channel_cmds,
                    duration_s=duration, settle_s=args.settle_s,
                    payload=payload, out_dir=out_dir,
                    run_idx=run_idx, total_runs=runs,
                    is_nb=args.nb)
                cell_runs.append(run)
                run_dt = time.time() - t_run
                # One-line per-run progress (the spec calls it out explicitly).
                print(f'  [CAL] cfg={cfg_id} ch={channel} r{run_idx}/{runs}: '
                      f'eff_bps={run["eff_bps"]:.0f} '
                      f'sack_rate={run["sack_rate"]:.2f} '
                      f'batches={run["batch_count"]} '
                      f'loss={run["frame_loss_pct"]:.1f}% '
                      f'conn={run["connected"]} '
                      f'break={run["break_fired"]} '
                      f'dt={run_dt:.0f}s '
                      f'err={run.get("error") or "-"}')

                # Persist intermediate aggregate after EVERY run (spec §
                # "Save intermediate JSON after EVERY cell" — we go finer:
                # after every run so a crash mid-cell still keeps data).
                # table_key is "table" for WB, "table_nb" for NB.
                out[table_key].setdefault(cfg_key, {})
                out[table_key][cfg_key][channel] = aggregate_cell(cell_runs)
                flush()
        except Exception as e:
            print(f'  [CAL] cell exception: {type(e).__name__}: {e}')
        finally:
            try:
                for rpi in ('rpi1', 'rpi2'):
                    b_send(bs, f'SSH {lid} {rpi} killall -9 mercury 2>/dev/null; '
                               f'echo done')
                b_send(bs, f'UNLOCK {lid}')
                bs.close()
            except Exception:
                pass

        # Cell summary.
        cell = out[table_key].get(cfg_key, {}).get(channel, {})
        if cell:
            print(f'  [CAL] cell SUMMARY cfg={cfg_id} ch={channel}: '
                  f'eff_bps mean={cell["eff_bps_mean"]} '
                  f'min={cell["eff_bps_min"]} max={cell["eff_bps_max"]} '
                  f'sigma={cell["eff_bps_sigma"]} '
                  f'sack_rate={cell["sack_rate_mean"]} '
                  f'loss={cell["frame_loss_pct"]}% '
                  f'n_ok={cell["n_runs"]} n_fail={cell["n_failed_runs"]} '
                  f'failed={cell["failed"]} break={cell["break_fired"]}')

    out['finished'] = time.strftime('%Y%m%d_%H%M%S')
    out['total_runtime_s'] = round(time.time() - wall_start, 1)
    flush()

    # Final summary.
    print(f'\n{"="*72}\n  CALIBRATION COMPLETE\n{"="*72}')
    print(f'  Output:         {args.out}')
    print(f'  Per-run logs:   {out_dir}')
    print(f'  Cells:          {out["total_cells"]}')
    print(f'  Runs:           {out["total_runs"]}')
    print(f'  Runtime:        {out["total_runtime_s"]/60:.1f} min '
          f'({out["total_runtime_s"]/3600:.2f} h)')
    print(f'  Total RX bytes: {out["total_rx_bytes"]:,}')
    print(f'  Mercury HEAD:   {out["mercury_head"]}')
    print(f'  Schema:         v{out["schema_version"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
