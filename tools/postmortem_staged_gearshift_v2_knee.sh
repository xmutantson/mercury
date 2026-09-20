#!/usr/bin/env bash
set -Eeuo pipefail

HOME_K=/home/kameron
export PYTHONPATH="$HOME_K${PYTHONPATH:+:$PYTHONPATH}"
ROOT="$(cat "$HOME_K/.quicksilver_gearshift_v2_knee_root")"

remote_run() {
    local pi="$1" cmd="$2" b64
    b64="$(printf '%s' "$cmd" | base64 -w0)"
    PI="$pi" CMD_B64="$b64" python3 - <<'PY'
import base64, os
import ionos_butler as B
out, rc = B._ssh_run(
    os.environ["PI"],
    base64.b64decode(os.environ["CMD_B64"]).decode(),
    timeout=120,
)
if out:
    print(out, end="" if out.endswith("\n") else "\n")
raise SystemExit(rc)
PY
}

echo "============================================================"
echo " Gearshift-v2 WGN25 canary read-only postmortem"
echo "============================================================"
echo "root: $ROOT"
echo

echo "=== canary process ==="
PID="$(cat "$ROOT/canary.pid" 2>/dev/null || true)"
echo "pid: ${PID:-none}"
if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
  ps -fp "$PID" || true
  echo "alive: yes"
else
  echo "alive: no"
fi

echo
echo "=== ACTIVE_ENV_VERIFIED.json ==="
[[ -f "$ROOT/ACTIVE_ENV_VERIFIED.json" ]] && jq . "$ROOT/ACTIVE_ENV_VERIFIED.json" || echo "missing"

echo
echo "=== runtime status ==="
S="$(ls "$ROOT"/runtime_status*.json 2>/dev/null | head -1 || true)"
[[ -n "$S" ]] && jq . "$S" || echo "missing"

echo
echo "=== FAILURE.json ==="
[[ -f "$ROOT/FAILURE.json" ]] && jq . "$ROOT/FAILURE.json" || echo "missing"

echo
echo "=== WGN25 window-01 checkpoint ==="
CP="$ROOT/checkpoints_quicksilver_gs2_canary/wgn25/window-01.json"
[[ -f "$CP" ]] && jq . "$CP" || echo "missing"

echo
echo "=== newest WGN25 raw attempts ==="
find "$ROOT/attempts_quicksilver_gs2_canary/wgn25" -maxdepth 1 -type f -name '*.json' -printf '%T@ %p\n' 2>/dev/null \
  | sort -nr | head -5 | while read -r _ f; do
      echo "--- $f"
      jq . "$f" || true
    done

echo
echo "=== progress tail ==="
tail -20 "$ROOT/progress_quicksilver_gs2_canary.jsonl" 2>/dev/null || true

echo
echo "=== canary.log tail ==="
tail -300 "$ROOT/canary.log" 2>/dev/null || true

echo
echo "=== Pi live process + recent logs ==="
for pi in rpi1 rpi2; do
  echo
  echo "===== $pi ====="
  remote_run "$pi" "
set +e
echo --- PROCESS ---
ps -ww -eo pid,ppid,lstart,etime,args | grep -E '[/]quicksilver-gs2-a01/mercury( |$)' || true
for p in \$(pgrep -f '[q]uicksilver-gs2-a01/mercury -m ARQ' 2>/dev/null); do
  echo --- PID \$p ---
  printf exe=; readlink -f /proc/\$p/exe
  printf sha=; sha256sum /proc/\$p/exe 2>/dev/null | cut -d ' ' -f1
  echo env:
  tr '\\0' '\\n' </proc/\$p/environ 2>/dev/null | grep -E '^MERCURY_(GEARSHIFT_V2|RATE_TABLE)=' || true
done
echo --- CMD TRACE ---
if test -f /tmp/mv2_cmd.log; then
  grep -E 'GEARSHIFT-V2|INBAND-TX|\\[GEARSHIFT\\] SET_CONFIG:|CONNECTED|DISCONNECT|BREAK|NACK|FAIL|ERROR' /tmp/mv2_cmd.log | tail -300 || true
fi
echo --- RSP TRACE ---
if test -f /tmp/mv2_rsp.log; then
  grep -E 'INBAND-RX|CONFIG_TAG|CONNECTED|DISCONNECT|BREAK|NACK|FAIL|ERROR' /tmp/mv2_rsp.log | tail -300 || true
fi
" || true
done
