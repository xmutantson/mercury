#!/bin/bash
# Parse P1 sweep CSVs -> cliff Es/N0 (first EsN0 with BER==0 reading down from
# high SNR, i.e. lowest EsN0 that still decodes clean) + net bps + SNR3k.
# SNR3k conversion (validated against baseline CONFIG_0 = +1.4 dB):
#   per-subcarrier post-EQ SNR at cliff ~= cliff_EsN0 + OFFSET (OFFSET~6.0 dB,
#   measured from modem pilot SNR estimate, ~mode-independent, 1:1 slope)
#   SNR3k = per_sc_SNR + 10log10(BW_occ/3000),  BW_occ = 46.875*Nc
OUT="${1:-p1_awgn}"
OFFSET="${OFFSET:-6.0}"
echo "OFFSET(per-sc SNR vs EsN0) = $OFFSET dB ; SNR3k = cliff + OFFSET + 10log10(46.875*Nc/3000)"
printf "%-22s %4s %8s %10s %12s %10s\n" "arm" "Nc" "netbps" "cliffEsN0" "SNR3k_dB" "BW_occHz"
for csv in "$OUT"/*.csv; do
  [ "$(basename "$csv")" = "progress.log" ] && continue
  arm=$(basename "$csv" .csv)
  meta="$OUT/${arm}_meta.txt"
  net=$(grep -oE "Bitrate: [0-9.]+" "$meta" 2>/dev/null | grep -oE "[0-9.]+" | head -1)
  nc=$(grep -oE "Nc=[0-9]+" "$meta" 2>/dev/null | grep -oE "[0-9]+" | head -1)
  [ -z "$nc" ] && nc=50
  # cliff = lowest EsN0 with BER==0 such that all higher are also 0 (clean waterfall).
  cliff=$(awk -F';' '
    /^-?[0-9.]+;[0-9.]/ {e=$1; b=$2; ber[e]=b; ord[++n]=e}
    END{
      # find lowest e (scanning all) where ber==0 and there is no e2>e with ber>0.5
      best="NA";
      for(i=1;i<=n;i++){ e=ord[i];
        if(ber[e]+0==0){ ok=1;
          for(j=1;j<=n;j++){ if(ord[j]+0 > e+0 && ber[ord[j]]+0 > 0.1){ ok=0 } }
          if(ok){ if(best=="NA" || e+0<best+0) best=e } } }
      print best
    }' "$csv")
  if [ "$cliff" = "NA" ] || [ -z "$cliff" ]; then
    snr3k="NA"; bw="NA"
  else
    bw=$(awk -v nc="$nc" 'BEGIN{printf "%.1f", 46.875*nc}')
    snr3k=$(awk -v c="$cliff" -v o="$OFFSET" -v nc="$nc" 'BEGIN{printf "%.2f", c+o+10*log(46.875*nc/3000)/log(10)}')
  fi
  printf "%-22s %4s %8s %10s %12s %10s\n" "$arm" "$nc" "${net:-NA}" "$cliff" "$snr3k" "$bw"
done
