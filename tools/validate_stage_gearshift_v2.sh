#!/usr/bin/env bash
# Resumable Gearshift-v2 validation + Pi staging.
# Run as: bash tools/validate_stage_gearshift_v2.sh
# Failure exits only this child Bash, never the caller's interactive shell.
# Does NOT overwrite installed/running modem binaries or stop the knee campaign.

set -Eeuo pipefail

REPO="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "$REPO" ]; then
    echo "ERROR: run this from inside the Mercury/Quicksilver git worktree."
    exit 1
fi
cd "$REPO"

CONTROL_HEAD="$(git rev-parse HEAD)"
WANT="${QUICKSILVER_RUNTIME_REF:-$(git rev-list -1 HEAD -- source include build.sh tests/cpu)}"
if [ -z "$WANT" ]; then
    echo "ERROR: could not resolve runtime-relevant Gearshift-v2 commit."
    exit 1
fi
SHORT="${WANT:0:8}"

trap 'rc=$?; printf "\n[FAIL] line %s, rc=%s\n" "$LINENO" "$rc"; exit "$rc"' ERR

echo "============================================================"
echo " Quicksilver Gearshift-v2 validation/staging"
echo " helper head:  $CONTROL_HEAD"
echo " runtime ref:  $WANT"
echo "============================================================"

# Reject tracked/staged source modifications, but preserve the bootstrap
# source-identity marker. That untracked text file is evidence, not build input.
if ! git diff --quiet -- || ! git diff --cached --quiet --; then
    echo "ERROR: tracked/staged worktree changes present; refusing ambiguous source."
    git status --short
    exit 1
fi

unexpected_untracked="$(git ls-files --others --exclude-standard | grep -vF 'GEARSHIFT_V2_SOURCE_IDENTITY.txt' || true)"
if [ -n "$unexpected_untracked" ]; then
    echo "ERROR: unexpected untracked files present; refusing ambiguous source."
    printf '%s\n' "$unexpected_untracked"
    exit 1
fi

if [ -f GEARSHIFT_V2_SOURCE_IDENTITY.txt ]; then
    echo "Preserving bootstrap identity marker: GEARSHIFT_V2_SOURCE_IDENTITY.txt"
fi

BRANCH="$(git branch --show-current)"
if [ "$BRANCH" != "gearshift-v2" ]; then
    echo "ERROR: expected branch gearshift-v2, got: ${BRANCH:-DETACHED}"
    exit 1
fi

echo
echo "=== 1. Local CPU validation ==="
if [ "${QUICKSILVER_SKIP_LOCAL_TESTS:-0}" = "1" ]; then
    echo "SKIP: local CPU validation explicitly skipped for resume"
else
    tests/cpu/run_gearshift_tests.sh
fi

echo
echo "=== 2. Runtime feature-gate inventory ==="
python3 tools/audit_default_off_features.py > "/tmp/quicksilver-default-off-${SHORT}.md"
echo "saved /tmp/quicksilver-default-off-${SHORT}.md"

echo
echo "=== 3. Locate ionos_butler ==="

if python3 -c 'import ionos_butler' >/dev/null 2>&1; then
    echo "ionos_butler already importable"
else
    IONOS_BUTLER_PY="${IONOS_BUTLER_PY:-}"
    if [ -z "$IONOS_BUTLER_PY" ]; then
        IONOS_BUTLER_PY="$(
            find "$HOME" -maxdepth 7 -type f -name ionos_butler.py -print -quit 2>/dev/null || true
        )"
    fi

    if [ -z "$IONOS_BUTLER_PY" ] || [ ! -f "$IONOS_BUTLER_PY" ]; then
        echo "ERROR: ionos_butler.py not found under $HOME"
        echo "Set IONOS_BUTLER_PY=/full/path/to/ionos_butler.py and rerun."
        exit 1
    fi

    IONOS_BUTLER_DIR="$(dirname "$IONOS_BUTLER_PY")"
    export PYTHONPATH="$IONOS_BUTLER_DIR${PYTHONPATH:+:$PYTHONPATH}"
    echo "ionos_butler: $IONOS_BUTLER_PY"

    python3 -c 'import ionos_butler; print("ionos_butler import PASS")'
fi

echo
echo "=== 4. Resume/build/stage on Pis ==="

WANT="$WANT" SHORT="$SHORT" python3 - <<'PY'
import base64
import os
import re
import shlex
import time

import ionos_butler as B

WANT = os.environ["WANT"]
SHORT = os.environ["SHORT"]

SRC2 = f"/home/rpi2/quicksilver-gs2-build-{SHORT}"
STAGE2 = "/home/rpi2/quicksilver-gs2-gearshift-v2"
STAGE1 = "/home/rpi1/quicksilver-gs2-gearshift-v2"
RPI2_ADDR = "192.168.2.215"
HTTP_PORT = 18765

def run(pi, cmd, timeout=300, check=True):
    out, rc = B._ssh_run(pi, cmd, timeout=timeout)
    out = out or ""
    if out:
        print(f"[{pi}] {out}", end="" if out.endswith("\n") else "\n")
    if check and rc != 0:
        raise RuntimeError(f"{pi}: rc={rc}: {cmd}")
    return out, rc

def bash(pi, script, timeout=300, check=True):
    payload = base64.b64encode(script.encode()).decode()
    command = f"printf %s {shlex.quote(payload)} | base64 -d | bash"
    return run(pi, command, timeout=timeout, check=check)


JOB_BASE = f"/tmp/quicksilver-gs2-build-{SHORT}"
JOB_SCRIPT = JOB_BASE + ".sh"
JOB_LOG = JOB_BASE + ".log"
JOB_STATUS = JOB_BASE + ".status"
JOB_PID = JOB_BASE + ".pid"


def quiet_run(pi, cmd, timeout=60):
    out, rc = B._ssh_run(pi, cmd, timeout=timeout)
    return (out or ""), rc


def detached_build(pi, script):
    """Start or reattach to a durable remote build and stream its log."""
    state_cmd = (
        f"if test -f {shlex.quote(JOB_STATUS)}; then "
        f"echo DONE:$(cat {shlex.quote(JOB_STATUS)}); "
        f"elif test -f {shlex.quote(JOB_PID)} && "
        f"kill -0 $(cat {shlex.quote(JOB_PID)}) 2>/dev/null; then "
        f"echo RUNNING:$(cat {shlex.quote(JOB_PID)}); "
        f"else echo ABSENT; fi"
    )
    state, _ = quiet_run(pi, state_cmd)
    state = state.strip().splitlines()[-1] if state.strip() else "ABSENT"

    launched_now = False
    if state == "ABSENT":
        payload = base64.b64encode(script.encode()).decode()
        launch = f"""
set -Eeuo pipefail
rm -f {shlex.quote(JOB_LOG)} {shlex.quote(JOB_STATUS)} {shlex.quote(JOB_PID)}
printf %s {shlex.quote(payload)} | base64 -d > {shlex.quote(JOB_SCRIPT)}
chmod 0700 {shlex.quote(JOB_SCRIPT)}
nohup bash {shlex.quote(JOB_SCRIPT)} > {shlex.quote(JOB_LOG)} 2>&1 </dev/null &
pid=$!
echo "$pid" > {shlex.quote(JOB_PID)}
echo "$pid"
"""
        out, _ = bash(pi, launch, timeout=30)
        pid_lines = re.findall(r"(?m)^\s*(\d+)\s*$", out)
        pid = pid_lines[-1] if pid_lines else "?"
        print(f"{pi}: detached native build started pid={pid}", flush=True)
        launched_now = True
        state = f"RUNNING:{pid}"
    elif state.startswith("RUNNING:"):
        print(
            f"{pi}: reattaching to existing native build "
            f"pid={state.split(':', 1)[1]}",
            flush=True,
        )
    elif state.startswith("DONE:"):
        print(
            f"{pi}: native build already completed; consuming saved result",
            flush=True,
        )

    count_out, _ = quiet_run(
        pi,
        f"test -f {shlex.quote(JOB_LOG)} && "
        f"wc -l < {shlex.quote(JOB_LOG)} || echo 0",
    )
    try:
        existing_lines = int(count_out.strip().splitlines()[-1])
    except Exception:
        existing_lines = 0

    # New launch: stream from line 1. Reattach/completed: show only recent context.
    last_line = 0 if launched_now else max(0, existing_lines - 40)

    try:
        while True:
            count_out, _ = quiet_run(
                pi,
                f"test -f {shlex.quote(JOB_LOG)} && "
                f"wc -l < {shlex.quote(JOB_LOG)} || echo 0",
            )
            try:
                line_count = int(count_out.strip().splitlines()[-1])
            except Exception:
                line_count = last_line

            if line_count > last_line:
                chunk, _ = quiet_run(
                    pi,
                    f"sed -n '{last_line + 1},{line_count}p' "
                    f"{shlex.quote(JOB_LOG)}",
                )
                if chunk:
                    print(
                        f"[{pi}:build] {chunk}",
                        end="" if chunk.endswith("\n") else "\n",
                        flush=True,
                    )
                last_line = line_count

            state, _ = quiet_run(pi, state_cmd)
            state = state.strip().splitlines()[-1] if state.strip() else "ABSENT"

            if state.startswith("DONE:"):
                try:
                    rc = int(state.split(":", 1)[1])
                except ValueError:
                    raise RuntimeError(f"{pi}: malformed build status: {state}")
                if rc != 0:
                    raise RuntimeError(
                        f"{pi}: detached native build/test failed rc={rc}; "
                        f"log={JOB_LOG}"
                    )
                print(f"{pi}: detached native build/test PASS", flush=True)
                return

            if state == "ABSENT":
                raise RuntimeError(
                    f"{pi}: detached build disappeared without status; "
                    f"log={JOB_LOG}"
                )

            time.sleep(5)

    except KeyboardInterrupt:
        print(
            "\nLocal watcher detached. The rpi2 native build continues "
            "independently under nohup. Rerun this helper to reattach.",
            flush=True,
        )
        raise SystemExit(130)


def remote_sha(pi, path):
    out, _ = run(
        pi,
        f"test -f {shlex.quote(path)} && sha256sum {shlex.quote(path)} || true",
        timeout=60,
        check=False,
    )
    m = re.search(r"\b([0-9a-f]{64})\b", out)
    return m.group(1) if m else None

def has_build_id(pi, path):
    _, rc = run(
        pi,
        f"test -x {shlex.quote(path)} && strings {shlex.quote(path)} | "
        f"grep -Fq {shlex.quote(SHORT)}",
        timeout=60,
        check=False,
    )
    return rc == 0

stage2_binary = STAGE2 + "/mercury"
stage1_binary = STAGE1 + "/mercury"
source_binary = SRC2 + "/mercury"
NEW2 = SRC2 + ".new"

# A Ctrl-C against an older foreground helper may have left a native build
# running on rpi2. Do not start a competing compiler storm. Wait for any build
# whose cwd is one of our Gearshift-v2 build workspaces, with visible progress.
legacy_cmd = r"""for p in /proc/[0-9]*; do
    pid=${p##*/}
    cwd=$(readlink "$p/cwd" 2>/dev/null || true)
    case "$cwd" in
      /home/rpi2/quicksilver-gs2-build-*)
        cmd=$(tr '\0' ' ' < "$p/cmdline" 2>/dev/null || true)
        case "$cmd" in
          *build.sh*|*make*|*g++*|*gcc*|*clang*|*cmake*|*ninja*)
            printf '%s|%s|%s\n' "$pid" "$cwd" "$cmd"
            ;;
        esac
        ;;
    esac
done"""

while True:
    legacy_out, _ = quiet_run("rpi2", legacy_cmd)
    legacy_lines = [x for x in legacy_out.splitlines() if x.strip()]
    # Ignore our own durable job if this script is reattaching to it.
    job_pid_out, _ = quiet_run(
        "rpi2",
        f"test -f {shlex.quote(JOB_PID)} && cat {shlex.quote(JOB_PID)} || true",
    )
    own_pid = job_pid_out.strip()
    own_running = False
    if own_pid:
        _, own_rc = quiet_run("rpi2", f"kill -0 {shlex.quote(own_pid)} 2>/dev/null")
        own_running = own_rc == 0

    if own_running:
        # The durable job's shell and compiler children all share SRC2/NEW2.
        # Exclude that whole workspace family from the legacy-build guard.
        legacy_lines = [
            x for x in legacy_lines
            if f"|{SRC2}|" not in x and f"|{NEW2}|" not in x
        ]
    else:
        legacy_lines = [
            x for x in legacy_lines
            if not own_pid or not x.startswith(own_pid + "|")
        ]

    if not legacy_lines:
        break
    print("rpi2: older native build still active; waiting rather than starting another:", flush=True)
    for line in legacy_lines[:8]:
        print("  " + line, flush=True)
    time.sleep(5)

# Adopt the newest inactive legacy clone if this runtime-ref workspace does not
# exist yet. This avoids downloading the repository again after helper-only
# revisions changed the old workspace suffix.
adopt_script = f"""
set -Eeuo pipefail
if ! test -d {shlex.quote(SRC2 + '/.git')} && ! test -d {shlex.quote(NEW2 + '/.git')}; then
    candidate=""
    for d in $(ls -1dt /home/rpi2/quicksilver-gs2-build-* 2>/dev/null || true); do
        test "$d" = {shlex.quote(SRC2)} && continue
        test "$d" = {shlex.quote(NEW2)} && continue
        if test -d "$d/.git"; then
            candidate="$d"
            break
        fi
    done
    if test -n "$candidate"; then
        echo "ADOPT:$candidate"
        mv "$candidate" {shlex.quote(NEW2)}
    fi
fi
"""
adopt_out, adopt_rc = quiet_run("rpi2", adopt_script)
if adopt_rc != 0:
    raise RuntimeError(f"rpi2: legacy clone adoption failed rc={adopt_rc}")
for line in adopt_out.splitlines():
    if line.startswith("ADOPT:"):
        print(
            "rpi2: adopting existing clone into stable runtime workspace: "
            + line.split(":", 1)[1],
            flush=True,
        )

s2_sha = remote_sha("rpi2", stage2_binary)
s2_valid = bool(s2_sha and has_build_id("rpi2", stage2_binary))
print(f"rpi2 initial staged valid={s2_valid} sha256={s2_sha}")

if not s2_valid:
    src_sha = remote_sha("rpi2", source_binary)
    src_valid = bool(src_sha and has_build_id("rpi2", source_binary))

    if src_valid:
        print("rpi2: reusing already-built native binary; staging only")
        bash("rpi2", f"""
set -Eeuo pipefail
install -d {shlex.quote(STAGE2)}
install -m 0755 {shlex.quote(source_binary)} {shlex.quote(stage2_binary + ".new")}
mv {shlex.quote(stage2_binary + ".new")} {shlex.quote(stage2_binary)}
""")
    else:
        out, _ = run(
            "rpi2",
            f"if test -d {shlex.quote(SRC2 + '/.git')}; then "
            f"git -C {shlex.quote(SRC2)} rev-parse HEAD; fi",
            timeout=60,
            check=False,
        )
        src_head = out.strip().splitlines()[-1] if out.strip() else ""

        new_out, _ = run(
            "rpi2",
            f"if test -d {shlex.quote(NEW2 + '/.git')}; then "
            f"git -C {shlex.quote(NEW2)} rev-parse HEAD; fi",
            timeout=60,
            check=False,
        )
        new_head = new_out.strip().splitlines()[-1] if new_out.strip() else ""

        if src_head == WANT:
            print("rpi2: source workspace already at target; resuming build/test")
            detached_build("rpi2", f"""
set -Eeuo pipefail
trap 'rc=$?; echo "$rc" > {shlex.quote(JOB_STATUS)}' EXIT
cd {shlex.quote(SRC2)}
test "$(git rev-parse HEAD)" = {shlex.quote(WANT)}
MERCURY_BUILD_JOBS=4 MERCURY_BUILD_ID={shlex.quote(SHORT)} bash ./build.sh o3 --test
test -x ./mercury
strings ./mercury | grep -Fq {shlex.quote(SHORT)}
install -d {shlex.quote(STAGE2)}
install -m 0755 ./mercury {shlex.quote(stage2_binary + ".new")}
mv {shlex.quote(stage2_binary + ".new")} {shlex.quote(stage2_binary)}
""")
        elif new_head:
            print(
                "rpi2: reusing existing failed-at-build clone; "
                "updating it to target and resuming native build/test"
            )
            detached_build("rpi2", f"""
set -Eeuo pipefail
trap 'rc=$?; echo "$rc" > {shlex.quote(JOB_STATUS)}' EXIT
cd {shlex.quote(NEW2)}
git fetch origin gearshift-v2
git checkout --detach {shlex.quote(WANT)}
test "$(git rev-parse HEAD)" = {shlex.quote(WANT)}
MERCURY_BUILD_JOBS=4 MERCURY_BUILD_ID={shlex.quote(SHORT)} bash ./build.sh o3 clean --test
test -x ./mercury
strings ./mercury | grep -Fq {shlex.quote(SHORT)}
cd /
rm -rf {shlex.quote(SRC2)}
mv {shlex.quote(NEW2)} {shlex.quote(SRC2)}
install -d {shlex.quote(STAGE2)}
install -m 0755 {shlex.quote(source_binary)} {shlex.quote(stage2_binary + ".new")}
mv {shlex.quote(stage2_binary + ".new")} {shlex.quote(stage2_binary)}
""")
        else:
            print("rpi2: creating clean source workspace for target commit")
            detached_build("rpi2", f"""
set -Eeuo pipefail
trap 'rc=$?; echo "$rc" > {shlex.quote(JOB_STATUS)}' EXIT
rm -rf {shlex.quote(NEW2)}
git clone --no-tags --branch gearshift-v2 --single-branch \
  https://github.com/xmutantson/mercury.git {shlex.quote(NEW2)}
cd {shlex.quote(NEW2)}
git checkout --detach {shlex.quote(WANT)}
test "$(git rev-parse HEAD)" = {shlex.quote(WANT)}
MERCURY_BUILD_JOBS=4 MERCURY_BUILD_ID={shlex.quote(SHORT)} bash ./build.sh o3 clean --test
test -x ./mercury
strings ./mercury | grep -Fq {shlex.quote(SHORT)}
cd /
rm -rf {shlex.quote(SRC2)}
mv {shlex.quote(NEW2)} {shlex.quote(SRC2)}
install -d {shlex.quote(STAGE2)}
install -m 0755 {shlex.quote(source_binary)} {shlex.quote(stage2_binary + ".new")}
mv {shlex.quote(stage2_binary + ".new")} {shlex.quote(stage2_binary)}
""")

    s2_sha = remote_sha("rpi2", stage2_binary)
    if not s2_sha or not has_build_id("rpi2", stage2_binary):
        raise RuntimeError("rpi2 did not produce a valid staged target binary")
else:
    print("rpi2: matching staged binary already present; skipping rebuild")

s2_sha = remote_sha("rpi2", stage2_binary)
s1_sha = remote_sha("rpi1", stage1_binary)
s1_valid = bool(s1_sha and s1_sha == s2_sha and has_build_id("rpi1", stage1_binary))

if s1_valid:
    print("rpi1: staged binary already byte-identical to rpi2; skipping transfer")
else:
    print("rpi1: copying tested staged binary from rpi2")
    out, _ = bash("rpi2", f"""
set -Eeuo pipefail
cd {shlex.quote(STAGE2)}
nohup python3 -m http.server {HTTP_PORT} --bind 0.0.0.0 \
  >/tmp/quicksilver-stage-http-{SHORT}.log 2>&1 </dev/null &
echo $!
""", timeout=30)

    pids = re.findall(r"(?m)^\s*(\d+)\s*$", out)
    if not pids:
        raise RuntimeError("could not determine temporary rpi2 HTTP server PID")
    server_pid = pids[-1]

    try:
        bash("rpi1", f"""
set -Eeuo pipefail
install -d {shlex.quote(STAGE1)}
curl -fS --retry 5 --retry-delay 1 --connect-timeout 5 \
  http://{RPI2_ADDR}:{HTTP_PORT}/mercury \
  -o {shlex.quote(stage1_binary + ".new")}
chmod 0755 {shlex.quote(stage1_binary + ".new")}
mv {shlex.quote(stage1_binary + ".new")} {shlex.quote(stage1_binary)}
""", timeout=300)
    finally:
        run(
            "rpi2",
            f"kill {shlex.quote(server_pid)} 2>/dev/null || true",
            timeout=30,
            check=False,
        )

s1_sha = remote_sha("rpi1", stage1_binary)
s2_sha = remote_sha("rpi2", stage2_binary)

if not s1_sha or s1_sha != s2_sha:
    raise RuntimeError(f"staged hash mismatch: rpi1={s1_sha} rpi2={s2_sha}")
if not has_build_id("rpi1", stage1_binary) or not has_build_id("rpi2", stage2_binary):
    raise RuntimeError("staged binary build-id check failed")

print()
print("============================================================")
print(" STAGING PASS")
print("============================================================")
print("commit :", WANT)
print("sha256 :", s1_sha)
print("rpi1   :", stage1_binary)
print("rpi2   :", stage2_binary)
print()
print("Installed/running binaries were NOT overwritten.")
print("Existing knee campaign was NOT stopped.")
PY

echo
echo "=== 5. Installed vs staged runtime inspection ==="
python3 - <<'PY'
import ionos_butler as B

for pi in ("rpi1", "rpi2"):
    installed = f"/home/{pi}/quicksilver-gs2-a01/mercury"
    staged = f"/home/{pi}/quicksilver-gs2-gearshift-v2/mercury"
    cmd = (
        "echo INSTALLED; "
        f"sha256sum {installed} 2>/dev/null || true; "
        "echo STAGED; "
        f"sha256sum {staged} 2>/dev/null || true; "
        "echo PROCESSES; "
        "ps -eo pid,lstart,args | grep '[m]ercury' || true"
    )
    out, _ = B._ssh_run(pi, cmd, timeout=60)
    print(f"\n===== {pi} =====")
    print(out or "", end="" if (out or "").endswith("\n") else "\n")
PY

echo
echo "=== 6. Existing knee campaign — inspection only ==="
if [ -f "$HOME/.quicksilver_gearshift_v2_knee_root" ]; then
    ROOT="$(cat "$HOME/.quicksilver_gearshift_v2_knee_root")"
    echo "KNEE_ROOT=$ROOT"
    find "$ROOT" -maxdepth 4 -type f -printf '%T@ %p\n' 2>/dev/null \
        | sort -nr | head -50 || true
else
    echo "No ~/.quicksilver_gearshift_v2_knee_root pointer found."
fi

echo
echo "--- local campaign-related processes ---"
ps -eo pid,lstart,args | grep -E '[q]uicksilver|[i]onos|[g]earshift|[k]nee' || true

echo
echo "============================================================"
echo " VALIDATION_AND_STAGE_COMPLETE"
echo " No running campaign or installed modem was modified."
echo "============================================================"
 || true
)"
if [ -n "$unexpected_untracked" ]; then
    echo "ERROR: unexpected untracked files present; refusing ambiguous source."
    printf '%s\n' "$unexpected_untracked"
    exit 1
fi

if [ -f GEARSHIFT_V2_SOURCE_IDENTITY.txt ]; then
    echo "Preserving bootstrap identity marker: GEARSHIFT_V2_SOURCE_IDENTITY.txt"
fi

BRANCH="$(git branch --show-current)"
if [ "$BRANCH" != "gearshift-v2" ]; then
    echo "ERROR: expected branch gearshift-v2, got: ${BRANCH:-DETACHED}"
    exit 1
fi

echo
echo "=== 1. Local CPU validation ==="
tests/cpu/run_gearshift_tests.sh

echo
echo "=== 2. Runtime feature-gate inventory ==="
python3 tools/audit_default_off_features.py > "/tmp/quicksilver-default-off-${SHORT}.md"
echo "saved /tmp/quicksilver-default-off-${SHORT}.md"

echo
echo "=== 3. Resume/build/stage on Pis ==="

WANT="$WANT" SHORT="$SHORT" python3 - <<'PY'
import base64
import os
import re
import shlex

import ionos_butler as B

WANT = os.environ["WANT"]
SHORT = os.environ["SHORT"]

SRC2 = f"/home/rpi2/quicksilver-gs2-build-{SHORT}"
STAGE2 = "/home/rpi2/quicksilver-gs2-gearshift-v2"
STAGE1 = "/home/rpi1/quicksilver-gs2-gearshift-v2"
RPI2_ADDR = "192.168.2.215"
HTTP_PORT = 18765

def run(pi, cmd, timeout=300, check=True):
    out, rc = B._ssh_run(pi, cmd, timeout=timeout)
    out = out or ""
    if out:
        print(f"[{pi}] {out}", end="" if out.endswith("\n") else "\n")
    if check and rc != 0:
        raise RuntimeError(f"{pi}: rc={rc}: {cmd}")
    return out, rc

def bash(pi, script, timeout=300, check=True):
    payload = base64.b64encode(script.encode()).decode()
    command = f"printf %s {shlex.quote(payload)} | base64 -d | bash"
    return run(pi, command, timeout=timeout, check=check)

def remote_sha(pi, path):
    out, _ = run(
        pi,
        f"test -f {shlex.quote(path)} && sha256sum {shlex.quote(path)} || true",
        timeout=60,
        check=False,
    )
    m = re.search(r"\b([0-9a-f]{64})\b", out)
    return m.group(1) if m else None

def has_build_id(pi, path):
    _, rc = run(
        pi,
        f"test -x {shlex.quote(path)} && strings {shlex.quote(path)} | "
        f"grep -Fq {shlex.quote(SHORT)}",
        timeout=60,
        check=False,
    )
    return rc == 0

stage2_binary = STAGE2 + "/mercury"
stage1_binary = STAGE1 + "/mercury"
source_binary = SRC2 + "/mercury"

s2_sha = remote_sha("rpi2", stage2_binary)
s2_valid = bool(s2_sha and has_build_id("rpi2", stage2_binary))
print(f"rpi2 initial staged valid={s2_valid} sha256={s2_sha}")

if not s2_valid:
    src_sha = remote_sha("rpi2", source_binary)
    src_valid = bool(src_sha and has_build_id("rpi2", source_binary))

    if src_valid:
        print("rpi2: reusing already-built native binary; staging only")
        bash("rpi2", f"""
set -Eeuo pipefail
install -d {shlex.quote(STAGE2)}
install -m 0755 {shlex.quote(source_binary)} {shlex.quote(stage2_binary + ".new")}
mv {shlex.quote(stage2_binary + ".new")} {shlex.quote(stage2_binary)}
""")
    else:
        out, _ = run(
            "rpi2",
            f"if test -d {shlex.quote(SRC2 + '/.git')}; then "
            f"git -C {shlex.quote(SRC2)} rev-parse HEAD; fi",
            timeout=60,
            check=False,
        )
        src_head = out.strip().splitlines()[-1] if out.strip() else ""

        if src_head == WANT:
            print("rpi2: source workspace already at target; resuming build/test")
            bash("rpi2", f"""
set -Eeuo pipefail
cd {shlex.quote(SRC2)}
test "$(git rev-parse HEAD)" = {shlex.quote(WANT)}
MERCURY_BUILD_JOBS=4 MERCURY_BUILD_ID={shlex.quote(SHORT)} bash ./build.sh o3 --test
test -x ./mercury
strings ./mercury | grep -Fq {shlex.quote(SHORT)}
install -d {shlex.quote(STAGE2)}
install -m 0755 ./mercury {shlex.quote(stage2_binary + ".new")}
mv {shlex.quote(stage2_binary + ".new")} {shlex.quote(stage2_binary)}
""")
        else:
            print("rpi2: creating clean source workspace for target commit")
            bash("rpi2", f"""
set -Eeuo pipefail
rm -rf {shlex.quote(SRC2 + ".new")}
git clone --no-tags --branch gearshift-v2 --single-branch \
  https://github.com/xmutantson/mercury.git {shlex.quote(SRC2 + ".new")}
cd {shlex.quote(SRC2 + ".new")}
git checkout --detach {shlex.quote(WANT)}
test "$(git rev-parse HEAD)" = {shlex.quote(WANT)}
MERCURY_BUILD_JOBS=4 MERCURY_BUILD_ID={shlex.quote(SHORT)} bash ./build.sh o3 clean --test
test -x ./mercury
strings ./mercury | grep -Fq {shlex.quote(SHORT)}
cd /
rm -rf {shlex.quote(SRC2)}
mv {shlex.quote(SRC2 + ".new")} {shlex.quote(SRC2)}
install -d {shlex.quote(STAGE2)}
install -m 0755 {shlex.quote(source_binary)} {shlex.quote(stage2_binary + ".new")}
mv {shlex.quote(stage2_binary + ".new")} {shlex.quote(stage2_binary)}
""")

    s2_sha = remote_sha("rpi2", stage2_binary)
    if not s2_sha or not has_build_id("rpi2", stage2_binary):
        raise RuntimeError("rpi2 did not produce a valid staged target binary")
else:
    print("rpi2: matching staged binary already present; skipping rebuild")

s2_sha = remote_sha("rpi2", stage2_binary)
s1_sha = remote_sha("rpi1", stage1_binary)
s1_valid = bool(s1_sha and s1_sha == s2_sha and has_build_id("rpi1", stage1_binary))

if s1_valid:
    print("rpi1: staged binary already byte-identical to rpi2; skipping transfer")
else:
    print("rpi1: copying tested staged binary from rpi2")
    out, _ = bash("rpi2", f"""
set -Eeuo pipefail
cd {shlex.quote(STAGE2)}
nohup python3 -m http.server {HTTP_PORT} --bind 0.0.0.0 \
  >/tmp/quicksilver-stage-http-{SHORT}.log 2>&1 </dev/null &
echo $!
""", timeout=30)

    pids = re.findall(r"(?m)^\s*(\d+)\s*$", out)
    if not pids:
        raise RuntimeError("could not determine temporary rpi2 HTTP server PID")
    server_pid = pids[-1]

    try:
        bash("rpi1", f"""
set -Eeuo pipefail
install -d {shlex.quote(STAGE1)}
curl -fS --retry 5 --retry-delay 1 --connect-timeout 5 \
  http://{RPI2_ADDR}:{HTTP_PORT}/mercury \
  -o {shlex.quote(stage1_binary + ".new")}
chmod 0755 {shlex.quote(stage1_binary + ".new")}
mv {shlex.quote(stage1_binary + ".new")} {shlex.quote(stage1_binary)}
""", timeout=300)
    finally:
        run(
            "rpi2",
            f"kill {shlex.quote(server_pid)} 2>/dev/null || true",
            timeout=30,
            check=False,
        )

s1_sha = remote_sha("rpi1", stage1_binary)
s2_sha = remote_sha("rpi2", stage2_binary)

if not s1_sha or s1_sha != s2_sha:
    raise RuntimeError(f"staged hash mismatch: rpi1={s1_sha} rpi2={s2_sha}")
if not has_build_id("rpi1", stage1_binary) or not has_build_id("rpi2", stage2_binary):
    raise RuntimeError("staged binary build-id check failed")

print()
print("============================================================")
print(" STAGING PASS")
print("============================================================")
print("commit :", WANT)
print("sha256 :", s1_sha)
print("rpi1   :", stage1_binary)
print("rpi2   :", stage2_binary)
print()
print("Installed/running binaries were NOT overwritten.")
print("Existing knee campaign was NOT stopped.")
PY

echo
echo "=== 4. Installed vs staged runtime inspection ==="
python3 - <<'PY'
import ionos_butler as B

for pi in ("rpi1", "rpi2"):
    installed = f"/home/{pi}/quicksilver-gs2-a01/mercury"
    staged = f"/home/{pi}/quicksilver-gs2-gearshift-v2/mercury"
    cmd = (
        "echo INSTALLED; "
        f"sha256sum {installed} 2>/dev/null || true; "
        "echo STAGED; "
        f"sha256sum {staged} 2>/dev/null || true; "
        "echo PROCESSES; "
        "ps -eo pid,lstart,args | grep '[m]ercury' || true"
    )
    out, _ = B._ssh_run(pi, cmd, timeout=60)
    print(f"\n===== {pi} =====")
    print(out or "", end="" if (out or "").endswith("\n") else "\n")
PY

echo
echo "=== 5. Existing knee campaign — inspection only ==="
if [ -f "$HOME/.quicksilver_gearshift_v2_knee_root" ]; then
    ROOT="$(cat "$HOME/.quicksilver_gearshift_v2_knee_root")"
    echo "KNEE_ROOT=$ROOT"
    find "$ROOT" -maxdepth 4 -type f -printf '%T@ %p\n' 2>/dev/null \
        | sort -nr | head -50 || true
else
    echo "No ~/.quicksilver_gearshift_v2_knee_root pointer found."
fi

echo
echo "--- local campaign-related processes ---"
ps -eo pid,lstart,args | grep -E '[q]uicksilver|[i]onos|[g]earshift|[k]nee' || true

echo
echo "============================================================"
echo " VALIDATION_AND_STAGE_COMPLETE"
echo " No running campaign or installed modem was modified."
echo "============================================================"
