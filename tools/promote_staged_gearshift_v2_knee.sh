#!/usr/bin/env bash
set -Eeuo pipefail

HOME_K=/home/kameron
export PYTHONPATH="$HOME_K${PYTHONPATH:+:$PYTHONPATH}"

BRANCH="${QUICKSILVER_GS2_BRANCH:-gearshift-v2-single-owner}"
CURRENT_BRANCH="$(git branch --show-current)"
[[ "$CURRENT_BRANCH" == "$BRANCH" ]] || {
    echo "ERROR: expected branch $BRANCH, got ${CURRENT_BRANCH:-DETACHED}" >&2
    exit 1
}
COMMIT="$(git rev-parse HEAD)"
TREE="$(git rev-parse HEAD^{tree})"
EXPECTED_SHA256=""

KNEE_STATE="$HOME_K/.quicksilver_gearshift_v2_knee_root"
SERVICE="quicksilver-gs2-overnight.service"
STAGED_REL="quicksilver-gs2-gearshift-v2/mercury"
RUNTIME_REL="quicksilver-gs2-a01/mercury"

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
    timeout=1800,
)
if out:
    print(out, end="" if out.endswith("\n") else "\n")
raise SystemExit(rc)
PY
}

archive_and_stop_current_knee() {
    [[ -s "$KNEE_STATE" ]] || return 0
    local root pid status lid stamp archive
    root="$(cat "$KNEE_STATE")"
    [[ -d "$root" ]] || return 0
    stamp="$(date -u +%Y%m%dT%H%M%SZ)"
    archive="$root/invalid_controller_transition_$stamp"
    mkdir -p "$archive"

    status="$(ls "$root"/runtime_status*.json 2>/dev/null | head -1 || true)"
    pid="$(cat "$root/canary.pid" 2>/dev/null || true)"
    lid=""
    [[ -n "$status" ]] && lid="$(jq -r '.lease_id // empty' "$status" 2>/dev/null || true)"

    echo "=== freeze current active-verified regression run ==="
    echo "root:    $root"
    echo "pid:     ${pid:-none}"
    echo "lease:   ${lid:-none}"
    echo "archive: $archive"

    for f in \
        "$root"/runtime_status*.json \
        "$root"/RESULT.md \
        "$root"/PLAN.json \
        "$root"/BRANCH_IDENTITY.txt \
        "$root"/ACTIVE_ENV_VERIFIED.json \
        "$root"/JOURNALS.txt \
        "$root"/canary.log \
        "$root"/FAILURE.json \
        "$root"/DONE*; do
        [[ -e "$f" ]] && cp -a "$f" "$archive/" || true
    done

    cat > "$archive/CLASSIFICATION.txt" <<EOF
classification=CONTROLLER_REGRESSION
source_commit=d4b052a4ca2d13d3fffaada85901762c6ccd7c0a
controller_mode=active-verified
symptom=WGN25 remained approximately CONFIG_0-class goodput (~147 B/min)
root_cause_fix_commit=$COMMIT
root_cause_fix_tree=$TREE
fixes=Gearshift-v2 ACTIVE uses production CONFIG_TAG unilateral transport; peer-follow config-discriminating SACK is the transition confirmation; matching NACK/re-tag exhaustion are terminal failure events; Patch-3 population pre-veto removed
disposition=preserved_not_scored
EOF

    # Stop wrapper now; do not wait another measurement window.
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        kill -KILL "$pid" 2>/dev/null || true
        sleep 1
    fi

    # Release bench and reap only this known modem runtime.
    if [[ -n "$lid" ]]; then
        ROOT="$root" LID="$lid" python3 - <<'PY' || true
import importlib.util, os
from pathlib import Path
root = Path(os.environ["ROOT"])
lid = os.environ["LID"]
p = root / "quicksilver_gs2_canary.py"
spec = importlib.util.spec_from_file_location("cleanup_canary", p)
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
bs = c.V.Butler()
try:
    try: print("kill_mercury:", c.V.kill_mercury(bs, lid))
    except Exception as e: print("kill_mercury exception:", repr(e))
    try: print("unlock:", c.V.b_send(bs, f"UNLOCK {lid}", 25))
    except Exception as e: print("unlock exception:", repr(e))
finally:
    bs.close()
PY
    fi

    for pi in rpi1 rpi2; do
        remote_run "$pi" "
set -e
pids=\$(pgrep -f '[q]uicksilver-gs2-a01/mercury -m ARQ' || true)
[ -z \"\$pids\" ] || kill -TERM \$pids 2>/dev/null || true
sleep 1
pids=\$(pgrep -f '[q]uicksilver-gs2-a01/mercury -m ARQ' || true)
[ -z \"\$pids\" ] || kill -KILL \$pids 2>/dev/null || true
"
    done

    # Stop only the journal followers named by this run, preserving logs.
    if [[ -f "$root/JOURNALS.txt" ]]; then
        local rsp cmd rspdir cmddir
        rsp="$(awk -F= '$1=="rpi1"{print substr($0,index($0,"=")+1)}' "$root/JOURNALS.txt")"
        cmd="$(awk -F= '$1=="rpi2"{print substr($0,index($0,"=")+1)}' "$root/JOURNALS.txt")"
        if [[ -n "$rsp" ]]; then
            rspdir="$(dirname "$rsp")"
            remote_run rpi1 "D='$rspdir'; [ ! -s \"\$D/rsp-tail.pid\" ] || kill \"\$(cat \"\$D/rsp-tail.pid\")\" 2>/dev/null || true"
        fi
        if [[ -n "$cmd" ]]; then
            cmddir="$(dirname "$cmd")"
            remote_run rpi2 "D='$cmddir'; [ ! -s \"\$D/cmd-tail.pid\" ] || kill \"\$(cat \"\$D/cmd-tail.pid\")\" 2>/dev/null || true"
        fi
    fi

    # Move only current scored products; nested earlier archives remain untouched.
    for name in \
        attempts_quicksilver_gs2_canary \
        checkpoints_quicksilver_gs2_canary \
        progress_quicksilver_gs2_canary.jsonl \
        deployment_gs2_canary.json; do
        [[ ! -e "$root/$name" ]] || mv "$root/$name" "$archive/"
    done

    rm -f \
        "$root"/runtime_status*.json \
        "$root"/RESULT.md \
        "$root"/FAILURE.json \
        "$root"/DONE* \
        "$root"/STOP_AFTER_* \
        "$root"/canary.log \
        "$root"/canary.pid \
        "$root"/ACTIVE_ENV_VERIFIED.json \
        "$root"/PLAN.json
}

echo
echo "============================================================"
echo " PROMOTE ALREADY-VALIDATED GEARSHIFT-V2 STAGE"
echo "============================================================"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"

R2_MD5="$(remote_run rpi2 "md5sum '/home/rpi2/$STAGED_REL' | cut -d ' ' -f1" | tail -1)"
R2_SHA="$(remote_run rpi2 "sha256sum '/home/rpi2/$STAGED_REL' | cut -d ' ' -f1" | tail -1)"
[[ "$R2_MD5" =~ ^[0-9a-f]{32}$ ]] || die "bad rpi2 MD5: $R2_MD5"
[[ "$R2_SHA" =~ ^[0-9a-f]{64}$ ]] || die "bad rpi2 SHA256: $R2_SHA"
remote_run rpi2 "grep -aFq '${COMMIT:0:8}' '/home/rpi2/$STAGED_REL'" || die "rpi2 staged build-id mismatch"
# The validator already built and tested this exact commit. Once its embedded
# build-id matches HEAD, the staged binary itself is the SHA256 authority.
EXPECTED_SHA256="$R2_SHA"

echo "tested staged ARM MD5:    $R2_MD5"
echo "tested staged ARM SHA256: $R2_SHA"

echo
echo "=== cross-Pi staged identity ==="
for pi in rpi1 rpi2; do
    remote_run "$pi" "
set -e
P='/home/$pi/$STAGED_REL'
test \"\$(md5sum \"\$P\" | cut -d ' ' -f1)\" = '$R2_MD5'
test \"\$(sha256sum \"\$P\" | cut -d ' ' -f1)\" = '$R2_SHA'
echo '$pi OK'
md5sum \"\$P\"
sha256sum \"\$P\"
"
done

echo
echo "=== stop old automation + preserve bad ACTIVE evidence ==="
systemctl --user disable "$SERVICE" >/dev/null 2>&1 || true
systemctl --user stop "$SERVICE" >/dev/null 2>&1 || true
[[ "$(systemctl --user is-active "$SERVICE" 2>/dev/null || true)" != active ]] || die "old overnight service is active"
archive_and_stop_current_knee

echo
echo "=== install tested binary on both live runtime paths ==="
for pi in rpi1 rpi2; do
    remote_run "$pi" "
set -euo pipefail
src='/home/$pi/$STAGED_REL'
dst='/home/$pi/$RUNTIME_REL'
test \"\$(md5sum \"\$src\" | cut -d ' ' -f1)\" = '$R2_MD5'
[ ! -f \"\$dst\" ] || cp -a \"\$dst\" \"\$dst.pre-${COMMIT:0:7}-$STAMP\"
install -m 0755 \"\$src\" \"\$dst.new\"
test \"\$(md5sum \"\$dst.new\" | cut -d ' ' -f1)\" = '$R2_MD5'
mv -f \"\$dst.new\" \"\$dst\"
echo '$pi runtime installed:'
md5sum \"\$dst\"
sha256sum \"\$dst\"
"
done

echo
echo "=== update existing knee harness identity, not modem source ==="
ROOT="$(cat "$KNEE_STATE")"
RUNNER="$ROOT/gearshift_v2_knee_canary.py"
[[ -f "$RUNNER" ]] || die "knee runner missing: $RUNNER"

RUNNER="$RUNNER" COMMIT="$COMMIT" TREE="$TREE" MD5="$R2_MD5" SHA256="$R2_SHA" STAGED_REL="$STAGED_REL" python3 - <<'PY'
from pathlib import Path
import os, re
p=Path(os.environ['RUNNER'])
s=p.read_text()
vals={
 'COMMIT': os.environ['COMMIT'],
 'TREE': os.environ['TREE'],
 'EXPECTED_MD5': os.environ['MD5'],
 'EXPECTED_SHA256': os.environ['SHA256'],
}
for name,val in vals.items():
    pat=rf'(?m)^{name}\s*=\s*"[^"]*"\s*$'
    s,n=re.subn(pat, f'{name} = "{val}"', s, count=1)
    if n != 1: raise SystemExit(f'{name}: expected one assignment, got {n}')
pat=r'(?m)^C\.PI_STAGED_BINARY\s*=\s*"[^"]*"\s*$'
s,n=re.subn(pat, f'C.PI_STAGED_BINARY = "~/{os.environ["STAGED_REL"]}"', s, count=1)
if n != 1: raise SystemExit(f'C.PI_STAGED_BINARY: expected one assignment, got {n}')
p.write_text(s)
PY
python3 -m py_compile "$RUNNER"

cat > "$ROOT/BRANCH_IDENTITY.txt" <<EOF
repo=xmutantson/mercury
branch=$BRANCH
commit=$COMMIT
tree=$TREE
arm_md5=$R2_MD5
arm_sha256=$R2_SHA
runtime_rel=$RUNTIME_REL
staged_rel=$STAGED_REL
campaign=gearshift-v2-knee
updated_utc=$STAMP
EOF

echo
echo "=== fresh journals ==="
JSTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
remote_run rpi1 "
set -euo pipefail
: > /tmp/mv2_rsp.log
D='/home/rpi1/quicksilver-gs2-logarchive/gearshift-v2-knee-${COMMIT:0:7}-$JSTAMP'
mkdir -p \"\$D\"
nohup tail -n +1 -F /tmp/mv2_rsp.log >> \"\$D/rsp-journal.log\" 2>> \"\$D/rsp-tail.err\" </dev/null &
echo \$! > \"\$D/rsp-tail.pid\"
echo \"RSP_JOURNAL_DIR=\$D\"
"
remote_run rpi2 "
set -euo pipefail
: > /tmp/mv2_cmd.log
D='/home/rpi2/quicksilver-gs2-logarchive/gearshift-v2-knee-${COMMIT:0:7}-$JSTAMP'
mkdir -p \"\$D\"
nohup tail -n +1 -F /tmp/mv2_cmd.log >> \"\$D/cmd-journal.log\" 2>> \"\$D/cmd-tail.err\" </dev/null &
echo \$! > \"\$D/cmd-tail.pid\"
echo \"CMD_JOURNAL_DIR=\$D\"
"
cat > "$ROOT/JOURNALS.txt" <<EOF
stamp=$JSTAMP
rpi1=/home/rpi1/quicksilver-gs2-logarchive/gearshift-v2-knee-${COMMIT:0:7}-$JSTAMP/rsp-journal.log
rpi2=/home/rpi2/quicksilver-gs2-logarchive/gearshift-v2-knee-${COMMIT:0:7}-$JSTAMP/cmd-journal.log
EOF

echo
echo "=== launch fresh WGN25 window 1 ==="
(
    cd "$ROOT"
    nohup python3 -u ./gearshift_v2_knee_canary.py > ./canary.log 2>&1 </dev/null &
    echo $! > ./canary.pid
)
PID="$(cat "$ROOT/canary.pid")"

# Hard gate: actual child processes must again prove ACTIVE env + new binary MD5.
# Bench startup can legitimately take >3 minutes before the modem children exist,
# so do not treat a 120 s preflight as a controller failure.
for n in $(seq 1 420); do
    kill -0 "$PID" 2>/dev/null || { tail -150 "$ROOT/canary.log" || true; die "canary exited"; }
    [[ -f "$ROOT/ACTIVE_ENV_VERIFIED.json" ]] && break
    if (( n % 15 == 0 )); then
        echo "waiting for ACTIVE verifier... ${n}s"
        tail -3 "$ROOT/canary.log" 2>/dev/null || true
    fi
    sleep 1
done
[[ -f "$ROOT/ACTIVE_ENV_VERIFIED.json" ]] || die "ACTIVE environment never verified after 420s"
jq -e --arg m "$R2_MD5" '.verified == true and .mode == "active" and .deploy_md5 == $m' \
    "$ROOT/ACTIVE_ENV_VERIFIED.json" >/dev/null || die "ACTIVE verifier identity mismatch"

echo
echo "=== prove actual child process env + executable on both Pis ==="
for pi in rpi1 rpi2; do
    remote_run "$pi" "
set -e
pids=\$(pgrep -f '[q]uicksilver-gs2-a01/mercury -m ARQ' || true)
test -n \"\$pids\"
ok=0
for p in \$pids; do
    if tr '\\0' '\\n' < /proc/\$p/environ 2>/dev/null | grep -Fxq 'MERCURY_GEARSHIFT_V2=active' \
       && tr '\\0' '\\n' < /proc/\$p/environ 2>/dev/null | grep -Fxq 'MERCURY_RATE_TABLE=/dev/null/gearshift-v2-knee-no-calibration.json'; then
        exe=\$(readlink -f /proc/\$p/exe)
        test \"\$exe\" = '/home/$pi/$RUNTIME_REL'
        test \"\$(sha256sum /proc/\$p/exe | cut -d ' ' -f1)\" = '$EXPECTED_SHA256'
        echo PID=\$p ACTIVE_ENV_PASS EXE=\$exe
        ok=1
    fi
done
test \"\$ok\" = 1
"
done

# Wait only until the first actual scored measurement phase is entered.
for n in $(seq 1 300); do
    S="$(ls "$ROOT"/runtime_status*.json 2>/dev/null | head -1 || true)"
    if [[ -n "$S" ]] && jq -e '.phase=="measurement-window" and .profile=="WGN" and .dial==25 and .window==1' "$S" >/dev/null 2>&1; then
        break
    fi
    kill -0 "$PID" 2>/dev/null || { tail -150 "$ROOT/canary.log" || true; die "canary exited before WGN25 measurement"; }
    if (( n % 15 == 0 )); then
        echo "waiting for WGN25 measurement-window... ${n}s"
        [[ -n "$S" ]] && jq '{phase,profile,dial,window,attempt}' "$S" 2>/dev/null || true
    fi
    sleep 1
done

echo
echo "============================================================"
echo " CONFIG_TAG ROOT FIX DEPLOYED + CANARY RUNNING"
echo "============================================================"
echo "commit: $COMMIT"
echo "tree:   $TREE"
echo "MD5:    $R2_MD5"
echo "SHA256: $EXPECTED_SHA256"
echo "root:   $ROOT"
echo "pid:    $PID"
echo
echo "status:"
jq . "$ROOT"/runtime_status*.json

echo
echo "ACTIVE verifier:"
jq . "$ROOT/ACTIVE_ENV_VERIFIED.json"

echo
echo "=== prove ACTIVE uses CONFIG_TAG transport, not on-wire SET_CONFIG ==="
# The decision/queue label still says SET_CONFIG because add_message_control(SET_CONFIG) is
# the historical software chokepoint.  The wire transport must be intercepted there.
TRANSPORT_OK=0
for _ in $(seq 1 180); do
    TRACE="$(remote_run rpi2 "grep -E 'GEARSHIFT-V2|INBAND-TX|\[GEARSHIFT\] SET_CONFIG:' /tmp/mv2_cmd.log 2>/dev/null | tail -160 || true")"
    if grep -Fq '[GEARSHIFT] SET_CONFIG: forward=16' <<<"$TRACE"; then
        printf '%s\n' "$TRACE"
        die "ACTIVE fell through to the obsolete on-wire SET_CONFIG builder for cfg16"
    fi
    if grep -Fq '[INBAND-TX] UNILATERAL CONFIG 0 -> 16' <<<"$TRACE" \
       && grep -Fq '[INBAND-TX] CONFIG_TAG passband emit cfg=16' <<<"$TRACE"; then
        TRANSPORT_OK=1
        break
    fi
    kill -0 "$PID" 2>/dev/null || { tail -150 "$ROOT/canary.log" || true; die "canary exited before CONFIG_TAG transition proof"; }
    sleep 1
done

printf '%s\n' "$TRACE" | tail -160
[[ "$TRANSPORT_OK" == 1 ]] || die "no proved CONFIG_0->16 CONFIG_TAG transition observed"

echo
echo "CONFIG_TAG transport proof: PASS"

echo
echo "=== prove peer-follow evidence, not local PHY load, closes the v2 switch ==="
FOLLOW_OK=0
for _ in $(seq 1 180); do
    TRACE="$(remote_run rpi2 "grep -E 'GEARSHIFT-V2|INBAND-TX|switch-inflight' /tmp/mv2_cmd.log 2>/dev/null | tail -220 || true")"
    if grep -Fq '[GEARSHIFT-V2] switch-inflight-expired' <<<"$TRACE"; then
        printf '%s\n' "$TRACE"
        die "v2 fell through to the last-resort wall-clock switch watchdog"
    fi
    if grep -Fq '[INBAND-TX] CONFIRMED followed CONFIG_16' <<<"$TRACE" \
       && grep -Fq '[GEARSHIFT-V2] probe-confirmed source=0 target=16' <<<"$TRACE"; then
        FOLLOW_OK=1
        break
    fi
    kill -0 "$PID" 2>/dev/null || { tail -150 "$ROOT/canary.log" || true; die "canary exited before peer-follow confirmation proof"; }
    sleep 1
done

printf '%s\n' "$TRACE" | tail -220
[[ "$FOLLOW_OK" == 1 ]] || die "CONFIG_TAG was emitted but peer-follow confirmation was not observed"

echo
echo "Peer-follow lifecycle proof: PASS"