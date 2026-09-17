#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
CXX="${CXX:-g++}"
OUT="${TMPDIR:-/tmp}/mercury-gearshift-cpu-tests"
mkdir -p "$OUT"

"$CXX" -std=c++14 -Wall -Wextra -Werror -Iinclude \
  tests/cpu/test_optimizer_metrics.cc \
  -o "$OUT/test_optimizer_metrics"
"$OUT/test_optimizer_metrics"

"$CXX" -std=c++14 -Wall -Wextra -Werror -Iinclude \
  tests/cpu/test_optimizer_geometry.cc \
  -o "$OUT/test_optimizer_geometry"
"$OUT/test_optimizer_geometry"

"$CXX" -std=c++14 -Wall -Wextra -Werror -Iinclude \
  tests/cpu/test_gearshift_quality_report.cc \
  -o "$OUT/test_gearshift_quality_report"
"$OUT/test_gearshift_quality_report"

"$CXX" -std=c++14 -Wall -Wextra -Werror -Iinclude \
  source/datalink_layer/rate_optimizer.cc \
  tests/cpu/test_rate_optimizer.cc \
  -o "$OUT/test_rate_optimizer"
"$OUT/test_rate_optimizer"

"$CXX" -std=c++14 -Wall -Wextra -Werror -Iinclude \
  source/datalink_layer/rate_optimizer.cc \
  tests/cpu/test_rate_optimizer_current_table.cc \
  -o "$OUT/test_rate_optimizer_current_table"
"$OUT/test_rate_optimizer_current_table"

"$CXX" -std=c++14 -Wall -Wextra -Werror -Iinclude \
  source/datalink_layer/rate_optimizer.cc \
  tests/cpu/test_gearshift_v2.cc \
  -o "$OUT/test_gearshift_v2"
"$OUT/test_gearshift_v2"

"$CXX" -std=c++14 -Wall -Wextra -Werror -Iinclude \
  source/datalink_layer/rate_optimizer.cc \
  tests/cpu/test_gearshift_v2_closed_loop.cc \
  -o "$OUT/test_gearshift_v2_closed_loop"
"$OUT/test_gearshift_v2_closed_loop"

python3 -m py_compile tools/sim/effective_rate_calibrate.py
python3 tests/cpu/test_calibration_v2.py
python3 tests/cpu/audit_gearshift_v2_spec.py
printf '%s\n' 'PASS gearshift CPU test suite'
