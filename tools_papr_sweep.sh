#!/bin/bash
# P1-A PAPR-cut sweep: CFG16 (32-QAM r0.875) coded-BER A/B over data PAPR cut x SNR ladder.
# Uses the SFO-GRID coded harness with the passband-loopback clip (MERCURY_SFO_GRID_PBLOOP=1)
# so peak_clip(data_papr_cut) actually runs. Aggregates codewords-decoded across seeds.
# Usage: tools_papr_sweep.sh <exe> <out.csv>
set -u
EXE="${1:?exe}"
OUT="${2:?out csv}"
SEEDS="${PAPR_SEEDS:-1 2 3 4 5 6 7 8}"
ESN0S="${PAPR_ESN0S:-14.0 14.5 15.0 15.5 16.0 16.5}"
CUTS="${PAPR_CUTS:-off 15 13 10 7}"     # 'off' == no clip baseline

echo "cut,esn0,seeds,cw_decoded,cw_total,frac_decoded" > "$OUT"
for cut in $CUTS; do
  for e in $ESN0S; do
    dec=0; tot=0
    for s in $SEEDS; do
      line=$(MERCURY_SFO_GRID=1 MERCURY_SFO_GRID_CODED=1 MERCURY_SFO_GRID_PBLOOP=1 \
             MERCURY_DATA_PAPR_CUT="$cut" MERCURY_SFO_GRID_ESN0="$e" MERCURY_SFO_GRID_SEED="$s" \
             "$EXE" -m PLOT_PASSBAND -s 16 2>&1 | grep -oE "codewords_decoded=[0-9]+/[0-9]+" | head -1)
      d=$(echo "$line" | grep -oE "=[0-9]+/" | tr -dc '0-9')
      t=$(echo "$line" | grep -oE "/[0-9]+"  | tr -dc '0-9')
      d=${d:-0}; t=${t:-0}
      dec=$((dec+d)); tot=$((tot+t))
    done
    frac=$(awk -v a="$dec" -v b="$tot" 'BEGIN{ printf (b>0)? "%.4f":"0", a/b }')
    nseed=$(echo $SEEDS | wc -w)
    echo "$cut,$e,$nseed,$dec,$tot,$frac" | tee -a "$OUT"
  done
done
