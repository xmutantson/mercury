#!/usr/bin/env python3
"""Focused pure-CPU tests for Gearshift-v2 calibration reducers."""
from pathlib import Path
import ast, re

src = Path('tools/sim/effective_rate_calibrate.py').read_text(encoding='utf-8')
tree = ast.parse(src)
wanted = {'parse_opt_batch_records', 'filter_primitive_records_for_config', 'build_cell_run', 'aggregate_cell'}
mod = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in wanted], type_ignores=[])
ns = {'re': re}
exec(compile(mod, '<calibration-functions>', 'exec'), ns)
parse_opt_batch_records = ns['parse_opt_batch_records']
filter_primitive_records_for_config = ns['filter_primitive_records_for_config']
build_cell_run = ns['build_cell_run']
aggregate_cell = ns['aggregate_cell']

log = '''
[OPT-BATCH] id=1 cfg=14 rate_valid=1 outcome_valid=1 cycle_ms=1000 app_bytes=0 transport_bytes=300 sent=30 acked=30 sack=0 failed=0 batch_size=30 snr=20.00 snr_age=0 sel=0.0200 lineage_valid=1 new_sent=24 repair_sent=6 cfg_gen=7 unit_id=1 dir=forward
[OPT-APP-COMMIT] id=1 cfg=14 app_bytes=600 unit_id=1
[OPT-BATCH] id=2 cfg=14 rate_valid=1 outcome_valid=1 cycle_ms=1000 app_bytes=0 transport_bytes=0 sent=30 acked=0 sack=0 failed=1 batch_size=30 snr=20.00 snr_age=0 sel=0.0200 lineage_valid=1 new_sent=30 repair_sent=0 cfg_gen=7 unit_id=1 dir=forward
[OPT-BATCH] id=3 cfg=100 rate_valid=1 outcome_valid=1 cycle_ms=1500 app_bytes=0 transport_bytes=100 sent=1 acked=1 sack=0 failed=0 batch_size=1 snr=20.00 snr_age=0 sel=0.0200 lineage_valid=1 new_sent=1 repair_sent=0 cfg_gen=8 unit_id=2 dir=forward
'''
r = parse_opt_batch_records(log)
assert len(r) == 3
assert r[0]['app_bytes'] == 600
assert r[0]['batch_size'] == 30
assert r[1]['failed'] == 1
assert r[0]['lineage_valid'] == 1 and r[0]['new_sent'] == 24 and r[0]['repair_sent'] == 6
assert r[0]['config_generation'] == 7 and r[0]['application_unit_id'] == 1 and r[0]['direction'] == 'forward'
fixed = filter_primitive_records_for_config(r, 14)
assert len(fixed) == 2 and all(x['cfg'] == 14 for x in fixed)


# The harness-to-run join must preserve every v2 primitive metric.  A previous
# implementation indexed opt_window_samples unconditionally (KeyError on v2
# primitive logs) and silently dropped SNR/selectivity/failure/batch context.
harvest = {
    'eff_bps': 2400.0, 'transport_bps': 1800.0,
    'eff_bps_source': 'opt_batch_application', 'sack_rate': 0.25,
    'failed_batch_rate': 0.125, 'frame_success': 0.875,
    'batch_count': 8, 'batch_size': 24.0, 'frame_loss_pct': 7.5,
    'selectivity': 0.123, 'snr_db': 18.5, 'primitive_records': 8,
    'primitive_records_total': 10, 'lineage_records': 8, 'repair_frame_rate': 0.125,
    'break_fired': False, 'mech_summary': {'x': 1},
}
raw = {'connected': True, 'error': None, 'rx_bytes': 99, 'measured_s': 10}
joined = build_cell_run(raw, harvest, 3)
assert joined['primitive_records'] == 8
assert joined['opt_window_samples'] == 0
assert joined['primitive_records_total'] == 10
assert joined['lineage_records'] == 8 and joined['repair_frame_rate'] == 0.125
assert joined['failed_batch_rate'] == 0.125
assert joined['frame_success'] == 0.875
assert joined['batch_size'] == 24.0
assert joined['selectivity'] == 0.123
assert joined['snr_db'] == 18.5

# A real zero-delivery run is a zero performance sample, not an invalid run.
runs = [
    {'connected': True, 'error': None, 'eff_bps': 8000.0, 'sack_rate': 0.0,
     'batch_count': 10, 'batch_size': 30.0, 'frame_loss_pct': 0.0,
     'failed_batch_rate': 0.0, 'frame_success': 1.0, 'selectivity': 0.02,
     'snr_db': 20.0, 'repair_frame_rate': 0.10, 'break_fired': False},
    {'connected': True, 'error': None, 'eff_bps': 0.0, 'sack_rate': 0.0,
     'batch_count': 10, 'batch_size': 10.0, 'frame_loss_pct': 0.0,
     'failed_batch_rate': 1.0, 'frame_success': 0.0, 'selectivity': 0.02,
     'snr_db': 20.0, 'repair_frame_rate': 0.30, 'break_fired': True},
]
a = aggregate_cell(runs)
assert a['n_runs'] == 2
assert a['n_zero_delivery_runs'] == 1
assert a['eff_bps_mean'] == 4000.0
assert abs(a['batch_size_mean'] - 20.0) < 1e-9
assert a['outcome_semantics'] == 'v2_all_valid_runs_including_zero'
assert abs(a['repair_frame_rate_mean'] - 0.20) < 1e-9
assert a['break_fired'] and abs(a['break_run_rate'] - 0.5) < 1e-9
print('PASS Gearshift-v2 calibration primitives + zero/batch context')
