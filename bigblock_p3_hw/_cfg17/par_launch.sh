#!/bin/bash
# Parallel CFG17 sweep launcher for one fleet node.
# Args: <out_csv> <parallelism> then channel-spec groups as:
#   "<chan>:<esn0-list>:<seed-list>:<arm-list>"  (space-separated tokens inside lists use '+')
# Generates the full cell job list and runs cell_runner.sh under xargs -P.
set -u
OUT="$1"; shift
P="$1"; shift
JOBS=$(mktemp)
for grp in "$@"; do
  IFS=':' read -r chan elist slist alist <<< "$grp"
  for e in ${elist//+/ }; do
    for s in ${slist//+/ }; do
      for a in ${alist//+/ }; do
        echo "${chan},${e},${s},${a}" >> "$JOBS"
      done
    done
  done
done
NJOB=$(wc -l < "$JOBS")
echo "[$(hostname)] $NJOB cells -> $OUT  (P=$P)" >&2
: > "$OUT"
cat "$JOBS" | xargs -P "$P" -I{} bash /tmp/cell_runner.sh "{}" >> "$OUT" 2>/dev/null
echo "DONE $(grep -vc DONE "$OUT") rows" >> "$OUT"
rm -f "$JOBS"
echo "[$(hostname)] complete: $(grep -vc DONE "$OUT") rows" >&2
