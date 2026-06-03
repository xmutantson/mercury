#!/bin/bash
# P1 ACQUISITION sweep: full acquire+sync+decode (MERCURY_P1_ACQ=1), real
# Schmidl-Cox detection + Moose CFO (NOT the perfect-sync BER harness).
# Reports BER per Es/N0; BER=0 means acquisition+decode both succeeded.
set -u
BIN=./mercury.exe
OUT="${OUT:-p1_acq}"
mkdir -p "$OUT"
GRID="${GRID:-6 4 2 0 -2 -3 -4 -5 -6}"
FRAMES="${FRAMES:-40}"
CFO="${CFO:-0}"
NC="${NC:-5}"
RATE="${RATE:-4}"
MOD="${MOD:-qpsk}"
NAME="${NAME:-acq_nc${NC}_r${RATE}_cfo${CFO}}"
log="$OUT/$NAME.csv"
echo "# arm=$NAME Nc=$NC rate=$RATE mod=$MOD cfo=${CFO}Hz frames=$FRAMES grid=[$GRID]" > "$log"
echo "# EsN0;BER" >> "$log"
for e in $GRID; do
  line=$(MERCURY_P1=1 MERCURY_P1_MOD=$MOD MERCURY_P1_RATE=$RATE MERCURY_P1_NC=$NC MERCURY_P1_ACQ=1 \
    timeout 200 "$BIN" -m PLOT_PASSBAND -s 0 -f "$CFO" --ber-esn0="$e" --ber-frames="$FRAMES" -x none -n 2>/dev/null \
    | grep -E "^-?[0-9.]+;[0-9.]" | tail -1)
  echo "$line" >> "$log"
  echo "  $NAME EsN0=$e -> $line"
done
echo "=== $NAME done -> $log ==="
