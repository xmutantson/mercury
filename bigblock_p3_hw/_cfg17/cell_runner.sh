#!/bin/bash
# Single CFG17 cell runner. Arg: "channel,esn0,seed,arm". Prints one CSV line.
# Designed to be fed to xargs -P for parallel fleet sweeping.
set -u
cd "$HOME/mercury"
BIN=./mercury
spec="$1"
IFS=',' read -r clab esn0 seed arm <<< "$spec"

CH=""
case "$clab" in
  clean)     CH="MERCURY_SFO_GRID_CHAN=0" ;;
  detfloor)  CH="MERCURY_SFO_GRID_CHAN=1" ;;
  watt-good) CH="MERCURY_SFO_GRID_CHAN=3 MERCURY_SFO_GRID_WATT_DEPTH_DB=2 MERCURY_SFO_GRID_WATT_FD_HZ=0.5" ;;
  watt-mod)  CH="MERCURY_SFO_GRID_CHAN=3 MERCURY_SFO_GRID_WATT_DEPTH_DB=4 MERCURY_SFO_GRID_WATT_FD_HZ=1.0" ;;
  watt-poor) CH="MERCURY_SFO_GRID_CHAN=3 MERCURY_SFO_GRID_WATT_DEPTH_DB=6 MERCURY_SFO_GRID_WATT_FD_HZ=1.5" ;;
esac
AR=""
case "$arm" in
  uniform)    AR="MERCURY_SFO_GRID_PCS=0 MERCURY_SFO_GRID_TURBO_ITERS=1 MERCURY_SFO_GRID_NVFIX=0" ;;
  stack)      AR="MERCURY_SFO_GRID_PCS=1 MERCURY_SFO_GRID_TURBO_SEED=tinterp MERCURY_SFO_GRID_TURBO_ITERS=4 MERCURY_SFO_GRID_NVFIX=0" ;;
  stacknvfix) AR="MERCURY_SFO_GRID_PCS=1 MERCURY_SFO_GRID_TURBO_SEED=tinterp MERCURY_SFO_GRID_TURBO_ITERS=4 MERCURY_SFO_GRID_NVFIX=1" ;;
  pasls)      AR="MERCURY_SFO_GRID_PCS=1 MERCURY_SFO_GRID_TURBO_ITERS=1 MERCURY_SFO_GRID_NVFIX=0" ;;
  genie)      AR="MERCURY_SFO_GRID_PCS=1 MERCURY_SFO_GRID_GENIE=1" ;;
esac
BASE="MERCURY_SFO_GRID=1 MERCURY_SFO_GRID_CODED=1 MERCURY_SFO_GRID_M64=1 MERCURY_SFO_GRID_NSYMB=60"
res=$(env $BASE $CH $AR MERCURY_SFO_GRID_ESN0=$esn0 MERCURY_SFO_GRID_SEED=$seed $BIN -m PLOT_PASSBAND -s 16 2>/dev/null \
  | grep -E '^\[SFO-GRID-CODED\]   codewords_decoded=' \
  | grep -oE 'codewords_decoded=[0-9]+/[0-9]+' | head -1 \
  | sed -E 's#codewords_decoded=([0-9]+)/([0-9]+)#\1,\2#')
[ -z "$res" ] && res="ERR,7"
echo "${clab},${esn0},${seed},${arm},${res}"
