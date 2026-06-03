#!/bin/bash
# P1 coherent mid-band prototype BER sweep driver (THROWAWAY).
# Runs single-point coded-BER probes across an Es/N0 grid for each arm and
# captures the modem's printed net bit rate (rbc) + Nc/bandwidth. AWGN by
# default; set FSEL=1 to enable the static 2-ray frequency-selective channel.
#
# Usage: ./p1_sweep.sh <out_dir>
#   env knobs forwarded to mercury via MERCURY_P1*; FSEL/FSEL_DELAY/FSEL_AMP
set -u
BIN=./mercury.exe
OUT="${1:-p1_results}"
mkdir -p "$OUT"

# Es/N0 grid (harness axis). Wide enough to bracket cliffs from deep to shallow.
GRID="${GRID:--6 -4 -2 0 2 4 6 8 10 12 14 16 18 20}"
FRAMES="${FRAMES:-40}"

FSELARGS=""
if [ "${FSEL:-0}" = "1" ]; then
  FSELARGS="--fsel-test=on"
fi

run_arm () {
  local name="$1"; shift
  local config="$1"; shift   # integer config id for -s
  local log="$OUT/$name.csv"
  echo "# arm=$name config=$config FSEL=${FSEL:-0} frames=$FRAMES env: P1=${MERCURY_P1:-} MOD=${MERCURY_P1_MOD:-} RATE=${MERCURY_P1_RATE:-} NC=${MERCURY_P1_NC:-} GI_MS=${MERCURY_P1_GI_MS:-}" > "$log"
  # First, a meta run to capture Bitrate/Nc/bandwidth (use a quick high-SNR point).
  "$BIN" -m PLOT_PASSBAND -s "$config" $FSELARGS --ber-esn0=20 --ber-frames=2 -x none -n \
     > "$OUT/${name}_meta.txt" 2>&1
  grep -E "Bitrate:|\[P1\]|\[PHY\] Config|Nc=|bandwidth|BW=" "$OUT/${name}_meta.txt" | head -20 >> "$log"
  echo "# EsN0;BER" >> "$log"
  for e in $GRID; do
    # Single-point: mercury prints "<esn0>;<ber>" on stdout.
    line=$("$BIN" -m PLOT_PASSBAND -s "$config" $FSELARGS --ber-esn0="$e" --ber-frames="$FRAMES" -x none -n 2>/dev/null | grep -E "^-?[0-9.]+;" | tail -1)
    echo "$line" >> "$log"
    echo "  $name EsN0=$e -> $line"
  done
  echo "=== $name done -> $log ==="
}

"$@"
