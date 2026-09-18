#!/usr/bin/env bash
set -Eeuo pipefail

HOME_K=/home/kameron
export PYTHONPATH="$HOME_K${PYTHONPATH:+:$PYTHONPATH}"
ROOT="$(cat "$HOME_K/.quicksilver_gearshift_v2_knee_root")"

echo "=== local promotion/canary status ==="
printf "root: "; echo "$ROOT"
printf "canary pid: "; cat "$ROOT/canary.pid" 2>/dev/null || echo none
if [[ -s "$ROOT/canary.pid" ]]; then
  PID="$(cat "$ROOT/canary.pid")"
  if kill -0 "$PID" 2>/dev/null; then echo "canary alive: yes"; else echo "canary alive: no"; fi
fi
echo
echo "--- ACTIVE_ENV_VERIFIED.json ---"
if [[ -f "$ROOT/ACTIVE_ENV_VERIFIED.json" ]]; then jq . "$ROOT/ACTIVE_ENV_VERIFIED.json"; else echo "not present yet"; fi
echo
echo "--- runtime status ---"
S="$(ls "$ROOT"/runtime_status*.json 2>/dev/null | head -1 || true)"
if [[ -n "$S" ]]; then jq . "$S"; else echo "no runtime status yet"; fi
echo
echo "--- canary log tail ---"
tail -100 "$ROOT/canary.log" 2>/dev/null || true
echo
echo "=== Pi modem process/env snapshot ==="
python3 - <<'PY'
import ionos_butler as B
for pi in ("rpi1","rpi2"):
    cmd = r'''
set +e
echo HOST=$(hostname)
ps -ww -eo pid,ppid,etime,args | grep -E "[/]quicksilver-gs2-a01/mercury( |$)" || true
for p in $(pgrep -f "[q]uicksilver-gs2-a01/mercury -m ARQ" 2>/dev/null); do
  echo "--- PID $p ---"
  printf "exe="; readlink -f /proc/$p/exe
  printf "sha="; sha256sum /proc/$p/exe 2>/dev/null | cut -d " " -f1
  echo "env:"
  tr "\0" "\n" </proc/$p/environ 2>/dev/null | grep -E "^MERCURY_(GEARSHIFT_V2|RATE_TABLE)=" || true
done
'''
    out, rc = B._ssh_run(pi, cmd, timeout=60)
    print(f"\n===== {pi} rc={rc} =====")
    print(out or "", end="" if (out or "").endswith("\n") else "\n")
PY
echo
echo "=== command trace tail ==="
python3 - <<'PY'
import ionos_butler as B
cmd = r"""grep -E "GEARSHIFT-V2|INBAND-TX|\[GEARSHIFT\] SET_CONFIG:|CONNECTED|DISCONNECT" /tmp/mv2_cmd.log 2>/dev/null | tail -120 || true"""
out, rc = B._ssh_run("rpi2", cmd, timeout=60)
print(out or "", end="" if (out or "").endswith("\n") else "\n")
PY
