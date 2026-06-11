#!/bin/bash
# P1-A knee-zoom: at the CFG16 viability knee, capture BOTH frame-decode count AND the
# continuous post-FEC info BER + LDPC iter_mean per cut, multi-seed. The continuous metrics
# expose clip distortion even where the binary decode count ties. Higher iter_mean / higher
# residual BER at fixed Es/N0 == more clip distortion the decoder must work through.
set -u
EXE="${1:?exe}"
OUT="${2:?out csv}"
SEEDS="${KNEE_SEEDS:-1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16}"
ESN0S="${KNEE_ESN0S:-15.0 15.25 15.5}"
CUTS="${KNEE_CUTS:-off 15 13 12 11 10 9 8 7}"

echo "cut,esn0,seeds,cw_decoded,cw_total,frac_decoded,mean_BER,mean_itermean" > "$OUT"
for cut in $CUTS; do
  for e in $ESN0S; do
    dec=0; tot=0; ber_sum=0; it_sum=0; n=0
    for s in $SEEDS; do
      o=$(MERCURY_SFO_GRID=1 MERCURY_SFO_GRID_CODED=1 MERCURY_SFO_GRID_PBLOOP=1 \
          MERCURY_DATA_PAPR_CUT="$cut" MERCURY_SFO_GRID_ESN0="$e" MERCURY_SFO_GRID_SEED="$s" \
          "$EXE" -m PLOT_PASSBAND -s 16 2>&1)
      cwl=$(echo "$o" | grep -oE "codewords_decoded=[0-9]+/[0-9]+" | head -1)
      d=$(echo "$cwl" | grep -oE "=[0-9]+/" | tr -dc '0-9'); t=$(echo "$cwl" | grep -oE "/[0-9]+" | tr -dc '0-9')
      ber=$(echo "$o" | grep -oE "post_FEC_info_BER=[0-9.eE+-]+" | head -1 | sed 's/.*=//')
      itm=$(echo "$o" | grep -oE "iter_mean=[0-9.eE+-]+" | head -1 | sed 's/.*=//')
      d=${d:-0}; t=${t:-0}; ber=${ber:-1}; itm=${itm:-0}
      dec=$((dec+d)); tot=$((tot+t))
      ber_sum=$(awk -v a="$ber_sum" -v b="$ber" 'BEGIN{print a+b}')
      it_sum=$(awk -v a="$it_sum" -v b="$itm" 'BEGIN{print a+b}')
      n=$((n+1))
    done
    frac=$(awk -v a="$dec" -v b="$tot" 'BEGIN{ printf (b>0)?"%.4f":"0", a/b }')
    mber=$(awk -v a="$ber_sum" -v b="$n" 'BEGIN{ printf "%.5f", (b>0)?a/b:0 }')
    mitm=$(awk -v a="$it_sum" -v b="$n" 'BEGIN{ printf "%.3f", (b>0)?a/b:0 }')
    echo "$cut,$e,$n,$dec,$tot,$frac,$mber,$mitm" | tee -a "$OUT"
  done
done
