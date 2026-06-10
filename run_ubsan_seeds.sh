#!/bin/bash
# run_ubsan_seeds.sh — drive N SIM_INPROC 2-instance CONNECT+OFDM cycles under
# the UBSan binary across varied seeds and SNRs, capturing every [UBSAN] report
# and every crash (non-zero exit / SIGSEGV / SIGILL) into the artifact dir.
#
# Args: $1 = binary (default mercury_ubsan_O1.exe)
#       $2 = N seeds (default 60)
#       $3 = payload bytes (default 1500 — enough to climb into OFDM, small enough to be fast)
#       $4 = per-run timeout seconds (default 90)
# Env:  SNRLIST="900 30 20 12 8"  (space-sep SNR3K values to rotate through)
set -u
cd "$(dirname "$0")"

BIN="${1:-mercury_ubsan_O1.exe}"
N="${2:-60}"
PAYLOAD="${3:-1500}"
TOUT="${4:-90}"
OUTDIR="x:/Storage/Documents/hermes and mercury/bigblock_p3_hw/_o3ub"
mkdir -p "$OUTDIR/runs"
SNRLIST="${SNRLIST:-900 30 20 14 10 8}"
read -r -a SNRS <<< "$SNRLIST"

SUMMARY="$OUTDIR/seed_run_summary.csv"
echo "idx,seed,snr3k,exit,signal_class,ubsan_hits,connected,delivered_bps,final_cfg" > "$SUMMARY"

crashes=0; ubsan_runs=0; connected_runs=0
for ((i=0; i<N; i++)); do
    seed=$(( 1000 + i*97 ))
    snr=${SNRS[$(( i % ${#SNRS[@]} ))]}
    out="$OUTDIR/runs/run_${i}_seed${seed}_snr${snr}.out"
    err="$OUTDIR/runs/run_${i}_seed${seed}_snr${snr}.err"
    MERCURY_SIM_2INST=1 \
      MERCURY_SIM2_SEED=$seed \
      MERCURY_SIM2_SNR3K=$snr \
      MERCURY_SIM2_PAYLOAD_BYTES=$PAYLOAD \
      MERCURY_SIM2_STALL_ITERS=40000 \
      MERCURY_SIM2_MAXITERS=2000000 \
      timeout "$TOUT" "./$BIN" -m SIM_INPROC -n > "$out" 2> "$err"
    rc=$?
    # signal class: 139=SIGSEGV(128+11), 132=SIGILL(128+4), 134=SIGABRT(128+6), 124=timeout
    sig="ok"
    case $rc in
        139) sig="SIGSEGV"; ((crashes++)) ;;
        132) sig="SIGILL";  ((crashes++)) ;;
        134) sig="SIGABRT"; ((crashes++)) ;;
        124) sig="TIMEOUT" ;;
        0)   sig="ok" ;;
        *)   sig="exit$rc"; [ $rc -gt 128 ] && { sig="sig$((rc-128))"; ((crashes++)); } ;;
    esac
    uh=$(grep -c "\[UBSAN\]" "$err" 2>/dev/null || echo 0)
    [ "$uh" -gt 0 ] && ((ubsan_runs++))
    conn=$(grep -c "connected=1" "$out" 2>/dev/null || echo 0)
    [ "$conn" -gt 0 ] && ((connected_runs++))
    dbps=$(grep -o "delivered_bps_sim=[0-9.]*" "$out" 2>/dev/null | head -1 | cut -d= -f2)
    fcfg=$(grep -o "final_cfg=[0-9]*" "$out" 2>/dev/null | head -1 | cut -d= -f2)
    echo "$i,$seed,$snr,$rc,$sig,$uh,$conn,${dbps:-NA},${fcfg:-NA}" >> "$SUMMARY"
    echo "[run $i] seed=$seed snr=$snr rc=$rc class=$sig ubsan=$uh conn=$conn"
    # keep only err files that have a UBSAN hit or a crash (bound disk use)
    if [ "$uh" = "0" ] && [ "$sig" = "ok" ]; then rm -f "$out" "$err"; fi
done

echo
echo "=== DONE: $N runs, crashes=$crashes, runs_with_ubsan=$ubsan_runs, connected=$connected_runs ==="
echo "summary: $SUMMARY"
