#!/usr/bin/env bash
set -Eeuo pipefail

HOME_K=/home/kameron
export PYTHONPATH="$HOME_K${PYTHONPATH:+:$PYTHONPATH}"
ROOT="$(cat "$HOME_K/.quicksilver_gearshift_v2_knee_root")"
IDENTITY="$ROOT/BRANCH_IDENTITY.txt"
[[ -f "$IDENTITY" ]] || { echo "ERROR: missing $IDENTITY" >&2; exit 1; }
COMMIT="$(sed -n 's/^commit=//p' "$IDENTITY" | head -1)"
EXPECTED_SHA256="$(sed -n 's/^arm_sha256=//p' "$IDENTITY" | head -1)"
[[ "$COMMIT" =~ ^[0-9a-f]{40}$ ]] || { echo "ERROR: bad commit in $IDENTITY" >&2; exit 1; }
[[ "$EXPECTED_SHA256" =~ ^[0-9a-f]{64}$ ]] || { echo "ERROR: bad arm_sha256 in $IDENTITY" >&2; exit 1; }
STATUS="$(ls "$ROOT"/runtime_status*.json 2>/dev/null | head -1 || true)"
PID="$(cat "$ROOT/canary.pid" 2>/dev/null || true)"
CHECKPOINT="$ROOT/checkpoints_quicksilver_gs2_canary/wgn25/window-01.json"

die() { echo "ERROR: $*" >&2; exit 1; }

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
echo " Attach to live Gearshift-v2 WGN25 root-fix canary"
echo "============================================================"
echo "root: $ROOT"
echo "pid:  ${PID:-none}"

[[ -n "$PID" ]] || die "no canary pid"
kill -0 "$PID" 2>/dev/null || die "canary pid $PID is not alive"
[[ -f "$ROOT/ACTIVE_ENV_VERIFIED.json" ]] || die "ACTIVE_ENV_VERIFIED.json missing"
jq -e --arg c "$COMMIT" '.verified == true and .mode == "active" and .source_commit == $c' \
  "$ROOT/ACTIVE_ENV_VERIFIED.json" >/dev/null || die "ACTIVE verifier is not valid for this runtime commit"

echo
echo "=== current canary status ==="
[[ -n "$STATUS" ]] && jq . "$STATUS" || true

echo
echo "=== prove live child env + exact executable on both Pis ==="
for pi in rpi1 rpi2; do
    remote_run "$pi" "
set -e
pids=\$(pgrep -f '[q]uicksilver-gs2-a01/mercury -m ARQ' || true)
test -n \"\$pids\"
ok=0
for p in \$pids; do
  if tr '\\0' '\\n' < /proc/\$p/environ 2>/dev/null | grep -Fxq 'MERCURY_GEARSHIFT_V2=active' \
     && tr '\\0' '\\n' < /proc/\$p/environ 2>/dev/null | grep -Fxq 'MERCURY_RATE_TABLE=/dev/null/gearshift-v2-knee-no-calibration.json'; then
    test \"\$(sha256sum /proc/\$p/exe | cut -d ' ' -f1)\" = '$EXPECTED_SHA256'
    echo \"$pi PID=\$p ACTIVE_SHA_PASS\"
    ok=1
  fi
done
test \"\$ok\" = 1
"
done

echo
echo "=== watch root-fix lifecycle (no restart) ==="
DECISION_OK=0
TRANSPORT_OK=0
FOLLOW_OK=0
TRACE=""

for n in $(seq 1 180); do
    TRACE="$(remote_run rpi2 "grep -E 'GEARSHIFT-V2|INBAND-TX|\\[GEARSHIFT\\] SET_CONFIG:' /tmp/mv2_cmd.log 2>/dev/null | tail -260 || true")"

    if grep -Fq '[GEARSHIFT] SET_CONFIG: forward=16' <<<"$TRACE"; then
        printf '%s\n' "$TRACE" | tail -260
        die "ACTIVE cfg16 transition fell through to obsolete on-wire SET_CONFIG"
    fi
    if grep -Fq '[GEARSHIFT-V2] switch-inflight-expired' <<<"$TRACE"; then
        printf '%s\n' "$TRACE" | tail -260
        die "cfg16 transition hit the last-resort switch watchdog"
    fi

    grep -Fq 'mode=active action=PROBE cfg=0 target=16' <<<"$TRACE" && DECISION_OK=1 || true
    if grep -Fq '[INBAND-TX] UNILATERAL CONFIG 0 -> 16' <<<"$TRACE" \
       && grep -Fq '[INBAND-TX] CONFIG_TAG passband emit cfg=16' <<<"$TRACE"; then
        TRANSPORT_OK=1
    fi
    if grep -Fq '[INBAND-TX] CONFIRMED followed CONFIG_16' <<<"$TRACE" \
       && grep -Fq '[GEARSHIFT-V2] probe-confirmed source=0 target=16' <<<"$TRACE"; then
        FOLLOW_OK=1
    fi

    if (( DECISION_OK && TRANSPORT_OK && FOLLOW_OK )); then
        echo "decision 0->16:       PASS"
        echo "CONFIG_TAG transport: PASS"
        echo "peer-follow confirm:  PASS"
        echo
        printf '%s\n' "$TRACE" | grep -E 'mode=active action=PROBE cfg=0 target=16|UNILATERAL CONFIG 0 -> 16|CONFIG_TAG passband emit cfg=16|CONFIRMED followed CONFIG_16|probe-confirmed source=0 target=16' | tail -40 || true
        echo
        echo "Root-fix lifecycle proof complete; canary remains running."
        exit 0
    fi

    if [[ -f "$CHECKPOINT" ]]; then
        echo
        echo "WGN25 window 1 completed before the full expected proof chain appeared:"
        jq . "$CHECKPOINT"
        echo
        printf '%s\n' "$TRACE" | tail -260
        die "window completed without complete 0->16 CONFIG_TAG peer-follow proof"
    fi

    kill -0 "$PID" 2>/dev/null || {
        echo "canary exited while waiting for root-fix lifecycle"
        tail -150 "$ROOT/canary.log" 2>/dev/null || true
        exit 1
    }

    if (( n % 6 == 0 )); then
        STATUS="$(ls "$ROOT"/runtime_status*.json 2>/dev/null | head -1 || true)"
        printf "waiting... %3ds  decision=%d transport=%d follow=%d" "$((n*5))" "$DECISION_OK" "$TRANSPORT_OK" "$FOLLOW_OK"
        if [[ -n "$STATUS" ]]; then
            printf "  phase=%s window=%s" \
              "$(jq -r '.phase // "?"' "$STATUS" 2>/dev/null)" \
              "$(jq -r '.window // "?"' "$STATUS" 2>/dev/null)"
        fi
        echo
        printf '%s\n' "$TRACE" | grep -E 'GEARSHIFT-V2|UNILATERAL CONFIG|CONFIG_TAG passband emit|CONFIRMED followed' | tail -8 || true
    fi

    sleep 5
done

echo
printf '%s\n' "$TRACE" | tail -260
die "timed out after 15 minutes waiting for complete root-fix lifecycle proof"
