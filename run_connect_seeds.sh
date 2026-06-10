#!/bin/bash
# run_connect_seeds.sh — FAST CONNECT-crash hunt. Each run is the default SIM2
# short-payload (19-byte) CONNECT+deliver cycle (~6s), driven across many seeds
# and SNRs under the UBSan binary. Directly stresses the CONNECT handshake — the
# exact symptom the project review records (~20-27% CONNECT crash at -O3).
set -u
cd "$(dirname "$0")"
BIN="${1:-mercury_ubsan_O3_glibcxx.exe}"
N="${2:-60}"
TOUT="${3:-40}"
OUTDIR="x:/Storage/Documents/hermes and mercury/bigblock_p3_hw/_o3ub"
mkdir -p "$OUTDIR/connect_runs"
SNRLIST="${SNRLIST:-900 60 40 30 25}"
read -r -a SNRS <<< "$SNRLIST"
SUMMARY="$OUTDIR/connect_run_summary.csv"
echo "idx,seed,snr3k,exit,signal_class,ubsan_hits,connected" > "$SUMMARY"
crashes=0; ubruns=0; conn=0
for ((i=0; i<N; i++)); do
    seed=$(( 31337 + i*131 ))
    snr=${SNRS[$(( i % ${#SNRS[@]} ))]}
    out="$OUTDIR/connect_runs/c_${i}_s${seed}_n${snr}.out"
    err="$OUTDIR/connect_runs/c_${i}_s${seed}_n${snr}.err"
    MERCURY_SIM_2INST=1 MERCURY_SIM2_SEED=$seed MERCURY_SIM2_SNR3K=$snr \
      MERCURY_SIM2_MAXITERS="${MAXITERS:-300}" \
      timeout "$TOUT" "./$BIN" -m SIM_INPROC -n > "$out" 2> "$err"
    rc=$?
    sig="ok"
    case $rc in
        139) sig="SIGSEGV"; crashes=$((crashes+1)) ;;
        132) sig="SIGILL";  crashes=$((crashes+1)) ;;
        134) sig="SIGABRT"; crashes=$((crashes+1)) ;;
        124) sig="TIMEOUT" ;;
        0)   sig="ok" ;;
        *)   if [ $rc -gt 128 ]; then sig="sig$((rc-128))"; crashes=$((crashes+1)); else sig="exit$rc"; fi ;;
    esac
    uh=$(grep -c "\[UBSAN\]" "$err" 2>/dev/null); uh=${uh//[!0-9]/}; uh=${uh:-0}
    [ "$uh" -gt 0 ] && ubruns=$((ubruns+1))
    c=$(grep -c "connected=1" "$out" 2>/dev/null); c=${c//[!0-9]/}; c=${c:-0}
    [ "$c" -gt 0 ] && conn=$((conn+1))
    echo "$i,$seed,$snr,$rc,$sig,$uh,$c" >> "$SUMMARY"
    echo "[c$i] seed=$seed snr=$snr rc=$rc $sig ubsan=$uh conn=$c"
    # keep only interesting runs
    if [ "$uh" = "0" ] && [ "$sig" = "ok" ]; then rm -f "$out" "$err"; fi
done
echo
echo "=== CONNECT SWEEP DONE: N=$N crashes=$crashes ubsan_runs=$ubruns connected=$conn ==="
echo "summary: $SUMMARY"
