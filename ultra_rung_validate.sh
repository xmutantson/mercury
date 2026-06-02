#!/bin/bash
# ULTRA rung cliff validation (P3b gate G2): sweep the PRODUCTION ULTRA configs
# (200-203, NOT the MERCURY_BAUD_MULT env instrument) and confirm each rung's
# AWGN + poor-fading cliff matches the fading-gate target depth. This proves the
# per-config Nfft path (ultra_baud_mult) reaches the depths end-to-end, on the
# pinned 3 kHz noise reference (baud_mult is set per-config, so passband_test_EsN0
# pins ref_bandwidth = bandwidth*baud_mult automatically — no env needed).
#
# Cliff = deepest EsN0 (SNR3k dB) with BER==0 contiguous to the top of the
# ascending sweep (same definition as fading_fast.sh). Kill at ACK_DETECT_TEST to
# skip the slow ACK Monte-Carlo. Hard cap per cell scales with Nfft (K=8 is slow).
set -u
BIN=./mercury.exe
OUT=/tmp/ultra_validate
mkdir -p "$OUT"
CSV="$OUT/ultra_cliffs.csv"
echo "config,K,Nfft,channel,doppler_Hz,delay_ms,cliff_dB" > "$CSV"

extract_cliff() {
  awk -F';' '
    /ACK_DETECT_TEST/ { stop=1 }
    stop==0 && $0 ~ /^-?[0-9]+;[0-9.eE+-]+$/ { n++; E[n]=$1; B[n]=$2 }
    END { c="NA";
      for (i=1;i<=n;i++) if (B[i]+0==0) { if(i==1){c=E[i];break} if(B[i-1]+0>0){c=E[i];break} }
      print c }' "$1"
}

# config -> K (mirror ultra_baud_mult)
kfor() { case "$1" in 200) echo 8;; 201) echo 4;; 202) echo 2;; 203) echo 1;; *) echo 1;; esac; }

run() {
  local cfg="$1" chan="$2" dop="$3" del="$4"
  local K; K=$(kfor "$cfg"); local Nfft=$((256*K))
  local log="$OUT/cfg${cfg}_${chan}.log"
  # cap: K=1/2 ~120s, K=4 ~300s, K=8 ~600s (Nfft=2048 block is slow)
  local cap=120; [ "$K" -ge 4 ] && cap=360; [ "$K" -ge 8 ] && cap=700
  echo ">>> config=$cfg K=$K Nfft=$Nfft chan=$chan dop=$dop del=$del cap=${cap}s"
  if [ "$chan" = "awgn" ]; then
    MERCURY_FADING=0 $BIN -m PLOT_PASSBAND -s "$cfg" -R -x wasapi -n > "$log" 2>&1 &
  else
    MERCURY_FADING=1 MERCURY_FADING_DOPPLER=$dop MERCURY_FADING_DELAY_MS=$del MERCURY_FADING_SEED=777 \
      $BIN -m PLOT_PASSBAND -s "$cfg" -R -x wasapi -n > "$log" 2>&1 &
  fi
  local pid=$!
  local waited=0
  while kill -0 "$pid" 2>/dev/null; do
    if grep -q "ACK_DETECT_TEST" "$log" 2>/dev/null; then kill -9 "$pid" 2>/dev/null; break; fi
    sleep 3; waited=$((waited+3))
    if [ "$waited" -ge "$cap" ]; then kill -9 "$pid" 2>/dev/null; break; fi
  done
  wait "$pid" 2>/dev/null
  local cliff; cliff=$(extract_cliff "$log")
  local baud; baud=$(grep -m1 "\[BAUD\]" "$log" 2>/dev/null | sed 's/.*-> //')
  echo "    cliff=${cliff} dB  (waited ${waited}s)  ${baud}"
  echo "${cfg},${K},${Nfft},${chan},${dop},${del},${cliff}" >> "$CSV"
}

# Shallow rungs first (fast); deepest (K=8) last. AWGN + poor (1Hz/2ms) — the two
# the fading gate reported. (moderate/edge optional; poor is the binding channel.)
for cfg in 203 202 201 200; do
  run "$cfg" awgn 0   0
  run "$cfg" poor 1.0 2.0
done

echo "=== ULTRA RUNG CLIFFS ==="
cat "$CSV"
echo ""
echo "Targets (fading-gate, poor): ULTRA_3(203)~-13 ULTRA_2(202)~-16 ULTRA_1(201)~-19/-20 ULTRA_0(200)~-21"
