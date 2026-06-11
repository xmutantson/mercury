#!/bin/bash
# test_clktx_rename.sh — C4 RENAME-ONLY smoke gate for the audioio clock-drift
# print tags (audioio.c). See MEMORY.md C4 item + SIMFIDELITY_ROOTCAUSE.md §1.
#
# ROOT CAUSE the rename addresses: the old [CLK-TX]/[CLK-RX] tags printed a
# PRODUCER-PUSH-RATE / window-quantization metric (swings ±2000 ppm in 10 s),
# NOT the codec crystal. True inter-Pi skew is ±8.16 ppm (tone method). The
# misnomer drove an 80x over-stated skew into 3 docs + a 670-ppm sim resampler.
#
# This is a RENAME-ONLY change (no behaviour change). The tags only print from
# the live audio TX/RX threads (hardware/soundcard required), so this smoke test
# asserts on the COMPILED BINARY's embedded string table instead of a live run:
#   - OLD tags MUST be ABSENT  (fail-before: they are present)
#   - NEW tags MUST be PRESENT
# This is deterministic and needs no audio device, IONOS, or RF.
#
# Usage: tools/test_clktx_rename.sh [path-to-mercury-binary]
#   Default binary: ./mercury.exe (Windows) or ./mercury (POSIX), relative to
#   the mercury/ repo root (parent of tools/).
#
# Exit 0 = PASS, non-zero = FAIL.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

BIN="${1:-}"
if [ -z "$BIN" ]; then
    if [ -f "$REPO_ROOT/mercury.exe" ]; then
        BIN="$REPO_ROOT/mercury.exe"
    elif [ -f "$REPO_ROOT/mercury" ]; then
        BIN="$REPO_ROOT/mercury"
    else
        echo "FAIL: no mercury binary found (looked for $REPO_ROOT/mercury.exe and $REPO_ROOT/mercury)."
        echo "      Build first: bash build.sh o3"
        exit 2
    fi
fi

if [ ! -f "$BIN" ]; then
    echo "FAIL: binary not found: $BIN"
    exit 2
fi

echo "=== C4 [CLK-TX]/[CLK-RX] rename smoke gate ==="
echo "Binary: $BIN"

# Extract printable strings from the binary. `strings` is in the MSYS2/Git-Bash
# binutils; fall back to a tr/grep filter if absent.
extract_strings() {
    if command -v strings >/dev/null 2>&1; then
        strings -n 4 "$1"
    else
        # Minimal portable fallback: keep runs of >=4 printable chars.
        LC_ALL=C tr -c '[:print:]' '\n' < "$1" | grep -E '.{4,}'
    fi
}

STR="$(extract_strings "$BIN")"

OLD_TAGS=("[CLK-TX]" "[CLK-RX]" "[CLK-TX-GLITCH]" "[CLK-RX-GLITCH]")
NEW_TAGS=("[TX-PUSH-RATE]" "[RX-DELIVER-RATE]" "[TX-PUSH-GLITCH]" "[RX-DELIVER-GLITCH]")

fail=0

echo "--- Asserting OLD tags are ABSENT ---"
for tag in "${OLD_TAGS[@]}"; do
    if printf '%s\n' "$STR" | grep -qF -- "$tag"; then
        echo "  FAIL: stale tag still present in binary: $tag"
        fail=1
    else
        echo "  ok:   absent: $tag"
    fi
done

echo "--- Asserting NEW tags are PRESENT ---"
for tag in "${NEW_TAGS[@]}"; do
    if printf '%s\n' "$STR" | grep -qF -- "$tag"; then
        echo "  ok:   present: $tag"
    else
        echo "  FAIL: new tag missing from binary: $tag"
        fail=1
    fi
done

if [ "$fail" -ne 0 ]; then
    echo "=== RESULT: FAIL ==="
    exit 1
fi
echo "=== RESULT: PASS ==="
exit 0
