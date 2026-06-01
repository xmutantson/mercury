#!/bin/bash
# FAST baud-scaling fading cliff sweep: run mercury PLOT_PASSBAND, but kill it as
# soon as the BER block finishes (the ACK_DETECT_TEST marker appears) so we don't
# pay for the slow ACK-detection Monte-Carlo we don't need. Cliff = deepest EsN0
# with BER==0 contiguous to the top of the (ascending) sweep.
set -u
BIN=./mercury.exe
OUT=/tmp/fading_results
mkdir -p "$OUT"
CSV="$OUT/cliffs_fast.csv"
echo "K,Nfft,channel,doppler_Hz,delay_ms,cliff_dB" > "$CSV"

extract_cliff() {
  awk -F';' '
    /ACK_DETECT_TEST/ { stop=1 }
    stop==0 && $0 ~ /^-?[0-9]+;[0-9.eE+-]+$/ { n++; E[n]=$1; B[n]=$2 }
    END { c="NA";
      for (i=1;i<=n;i++) if (B[i]+0==0) { if(i==1){c=E[i];break} if(B[i-1]+0>0){c=E[i];break} }
      print c }' "$1"
}

run() {
  local K="$1" chan="$2" dop="$3" del="$4"
  local Nfft=$((256*K)); local log="$OUT/fast_K${K}_${chan}.log"
  echo ">>> K=$K Nfft=$Nfft chan=$chan dop=$dop del=$del"
  if [ "$chan" = "awgn" ]; then
    MERCURY_BAUD_MULT=$K MERCURY_FADING=0 $BIN -m PLOT_PASSBAND -s 100 -R -x wasapi -n > "$log" 2>&1 &
  else
    MERCURY_BAUD_MULT=$K MERCURY_FADING=1 MERCURY_FADING_DOPPLER=$dop MERCURY_FADING_DELAY_MS=$del MERCURY_FADING_SEED=777 \
      $BIN -m PLOT_PASSBAND -s 100 -R -x wasapi -n > "$log" 2>&1 &
  fi
  local pid=$!
  # Wait for the BER block to finish (ACK_DETECT_TEST marker) then kill, or until
  # the process exits, or a hard 500s cap.
  local waited=0
  while kill -0 "$pid" 2>/dev/null; do
    if grep -q "ACK_DETECT_TEST" "$log" 2>/dev/null; then kill -9 "$pid" 2>/dev/null; break; fi
    sleep 2; waited=$((waited+2))
    if [ "$waited" -ge 500 ]; then kill -9 "$pid" 2>/dev/null; break; fi
  done
  wait "$pid" 2>/dev/null
  local cliff=$(extract_cliff "$log")
  echo "    cliff=${cliff} dB (waited ${waited}s)"
  echo "${K},${Nfft},${chan},${dop},${del},${cliff}" >> "$CSV"
}

# Full matrix (re-do all so the fast CSV is self-consistent; K1/K2 are quick).
for K in 1 2 4 8; do
  run "$K" awgn     0   0
  run "$K" moderate 0.5 1.0
  run "$K" poor     1.0 2.0
  run "$K" pooredge 2.0 2.0
done

echo "=== FAST RESULTS ==="
cat "$CSV"
