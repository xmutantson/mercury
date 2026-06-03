#!/bin/bash
set -u
BIN=./mercury.exe
OUT="${1:-p1_awgn}"
mkdir -p "$OUT"
GRID="${GRID:--8 -6 -4 -2 0 2 4 6 8 10 12 14 16}"
FRAMES="${FRAMES:-40}"
FSELARGS=""
[ "${FSEL:-0}" = "1" ] && FSELARGS="--fsel-test=on"

run_arm () {
  local name="$1"; local env="$2"
  local log="$OUT/$name.csv"
  echo "# arm=$name env=[$env] FSEL=${FSEL:-0} frames=$FRAMES grid=[$GRID]" > "$log"
  env $env "$BIN" -m PLOT_PASSBAND -s 0 $FSELARGS --ber-esn0=18 --ber-frames=2 > "$OUT/${name}_meta.txt" 2>&1
  grep -E "Bitrate:|\[P1\]|Config 0 active|Shannon_limit" "$OUT/${name}_meta.txt" | head -8 >> "$log"
  echo "# EsN0;BER" >> "$log"
  for e in $GRID; do
    line=$(env $env "$BIN" -m PLOT_PASSBAND -s 0 $FSELARGS --ber-esn0="$e" --ber-frames="$FRAMES" 2>/dev/null | grep -E "^-?[0-9.]+;[0-9.]" | tail -1)
    echo "$line" >> "$log"
  done
  echo "DONE $name" >> "$OUT/progress.log"
}

echo "START $(date)" > "$OUT/progress.log"
run_arm "baseline_cfg0_nc50"      ""
run_arm "p1_qpsk_r4_nc5"          "MERCURY_P1=1 MERCURY_P1_MOD=qpsk MERCURY_P1_RATE=4 MERCURY_P1_NC=5"
run_arm "p1_qpsk_r5_nc5"          "MERCURY_P1=1 MERCURY_P1_MOD=qpsk MERCURY_P1_RATE=5 MERCURY_P1_NC=5"
run_arm "p1_qpsk_r6_nc5"          "MERCURY_P1=1 MERCURY_P1_MOD=qpsk MERCURY_P1_RATE=6 MERCURY_P1_NC=5"
run_arm "p1_qpsk_r4_nc4"          "MERCURY_P1=1 MERCURY_P1_MOD=qpsk MERCURY_P1_RATE=4 MERCURY_P1_NC=4"
run_arm "p1_qpsk_r4_nc8"          "MERCURY_P1=1 MERCURY_P1_MOD=qpsk MERCURY_P1_RATE=4 MERCURY_P1_NC=8"
run_arm "p1_bpsk_r2_nc5"          "MERCURY_P1=1 MERCURY_P1_MOD=bpsk MERCURY_P1_RATE=2 MERCURY_P1_NC=5"
echo "ALLDONE $(date)" >> "$OUT/progress.log"
