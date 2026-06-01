#!/bin/bash
# Baud-scaling fading cliff sweep (baud-fading-spike.md).
# Runs the ROBUST_0 MFSK cliff sweep for each baud multiplier K under:
#   - AWGN only (fading off)         [reproduces P0 baseline]
#   - moderate fading (0.5 Hz / 1 ms)
#   - poor fading     (1.0 Hz / 2 ms)
#   - poor-edge       (2.0 Hz / 2 ms)   (stress the deepest rung)
# Pins the 3 kHz noise reference (P0 §2: baud_mult multiplies the ref bandwidth
# back, so the SNR axis is a fixed SNR3k for every K — handled in the binary).
# Cliff = deepest EsN0 (dB) with BER==0 such that the row BELOW it has BER>0.
set -u
BIN=./mercury.exe
OUT=/tmp/fading_results
mkdir -p "$OUT"
RESULT_CSV="$OUT/cliffs.csv"
echo "K,Nfft,channel,doppler_Hz,delay_ms,cliff_dB,note" > "$RESULT_CSV"

# Extract cliff from a sweep log: take the FIRST BER block (EsN0;BER rows before
# the ACK_DETECT_TEST marker). Cliff = lowest EsN0 with BER==0 contiguous to the
# top (i.e. the deepest dB that still decodes perfectly).
extract_cliff() {
  local log="$1"
  awk -F';' '
    /ACK_DETECT_TEST/ { stop=1 }
    stop==0 && $0 ~ /^-?[0-9]+;[0-9.eE+-]+$/ {
      esn0=$1; ber=$2; n++; E[n]=esn0; B[n]=ber;
    }
    END {
      # rows are in ascending EsN0. find the lowest esn0 index where BER==0 AND
      # the next-lower row (index-1) has BER>0 (the cliff edge). If BER==0 at the
      # very lowest row, report that row (cliff at/below sweep floor).
      cliff="NA";
      for (i=1; i<=n; i++) {
        if (B[i]+0==0) {
          if (i==1) { cliff=E[i]; break; }
          if (B[i-1]+0 > 0) { cliff=E[i]; break; }
        }
      }
      print cliff;
    }' "$log"
}

run_one() {
  local K="$1" chan="$2" dop="$3" del="$4" note="$5"
  local Nfft=$((256*K))
  local tag="K${K}_${chan}"
  local log="$OUT/${tag}.log"
  echo ">>> K=$K (Nfft=$Nfft) chan=$chan dop=$dop del=$del"
  if [ "$chan" = "awgn" ]; then
    MERCURY_BAUD_MULT=$K MERCURY_FADING=0 \
      timeout 600 $BIN -m PLOT_PASSBAND -s 100 -R -x wasapi -n > "$log" 2>&1
  else
    MERCURY_BAUD_MULT=$K MERCURY_FADING=1 MERCURY_FADING_DOPPLER=$dop MERCURY_FADING_DELAY_MS=$del MERCURY_FADING_SEED=777 \
      timeout 600 $BIN -m PLOT_PASSBAND -s 100 -R -x wasapi -n > "$log" 2>&1
  fi
  local rc=$?
  local cliff=$(extract_cliff "$log")
  echo "    cliff=${cliff} dB (rc=$rc)"
  echo "${K},${Nfft},${chan},${dop},${del},${cliff},${note}" >> "$RESULT_CSV"
}

for K in 1 2 4 8; do
  run_one "$K" awgn     0   0   "baseline_AWGN"
  run_one "$K" moderate 0.5 1.0 "ITU_moderate"
  run_one "$K" poor     1.0 2.0 "ITU_poor"
  run_one "$K" pooredge 2.0 2.0 "stress_2Hz"
done

echo "=== RESULTS ==="
cat "$RESULT_CSV"
