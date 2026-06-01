#!/bin/bash
# High-frame confirmation of the deep-rung fading cliffs (baud-fading-spike.md §5).
# The 3-frame sweep has ~+/-1 dB cliff variance and fading adds rare-deep-fade noise.
# This probes each baud at SNRs bracketing its cliff with MANY frames per point so
# the BER averages over many fading realizations => low-variance cliff location.
# Decision-critical for K=4/K=8 (they set ULTRA_0's fading floor).
set -u
BIN=./mercury.exe
FR=40                      # frames per point (vs 3 in the sweep)
OUT=/tmp/fading_confirm
mkdir -p "$OUT"
CSV="$OUT/confirm.csv"
echo "K,channel,doppler,delay_ms,esn0_dB,frames,BER" > "$CSV"

probe() {
  local K="$1" chan="$2" dop="$3" del="$4" esn0="$5"
  local log="$OUT/K${K}_${chan}_${esn0}.log"
  local env_fade=""
  if [ "$chan" = "awgn" ]; then
    MERCURY_BAUD_MULT=$K MERCURY_FADING=0 \
      timeout 400 $BIN -m PLOT_PASSBAND -s 100 -R --ber-esn0=$esn0 --ber-frames=$FR -x wasapi -n > "$log" 2>&1
  else
    MERCURY_BAUD_MULT=$K MERCURY_FADING=1 MERCURY_FADING_DOPPLER=$dop MERCURY_FADING_DELAY_MS=$del MERCURY_FADING_SEED=4242 \
      timeout 400 $BIN -m PLOT_PASSBAND -s 100 -R --ber-esn0=$esn0 --ber-frames=$FR -x wasapi -n > "$log" 2>&1
  fi
  local ber=$(grep -E "^-?[0-9.]+;[0-9.eE+-]+$" "$log" | tail -1 | cut -d';' -f2)
  echo "    K=$K $chan esn0=$esn0 BER=$ber"
  echo "${K},${chan},${dop},${del},${esn0},${FR},${ber}" >> "$CSV"
}

# Brackets per baud: a few dB around the AWGN cliff (K1 -13, K2 -16, K4 -20, K8 ~-22..-24).
# AWGN re-probe (high-frame) + poor + pooredge at the same SNRs => direct fading penalty.
probe_baud() {
  local K="$1"; shift
  local snrs=("$@")
  for s in "${snrs[@]}"; do
    probe "$K" awgn     0   0   "$s"
    probe "$K" poor     1.0 2.0 "$s"
    probe "$K" pooredge 2.0 2.0 "$s"
  done
}

probe_baud 1 -11 -12 -13 -14
probe_baud 2 -14 -15 -16 -17
probe_baud 4 -18 -19 -20 -21
probe_baud 8 -20 -21 -22 -23 -24

echo "=== CONFIRM RESULTS ==="
cat "$CSV"
