#!/bin/bash
# CFG17 fleet validation sweep driver — runs on a fleet node, emits CSV to stdout.
# Arms: uniform-64 LS-only / CFG17-full-stack (PAS+TINTERP-seed-turbo+nvfix) / genie-PAS.
# Channels passed as the first arg (a label); the per-channel env is selected below.
# CSV columns: channel,esn0,seed,arm,ok,tot
set -u
cd "$HOME/mercury"
BIN=./mercury

final() {
  grep -E '^\[SFO-GRID-CODED\]   codewords_decoded=' \
    | grep -oE 'codewords_decoded=[0-9]+/[0-9]+' | head -1 \
    | sed -E 's#codewords_decoded=([0-9]+)/([0-9]+)#\1,\2#'
}

# $1 channel-label  $2 esn0  $3 seed  $4 arm
run_cell() {
  local clab="$1" esn0="$2" seed="$3" arm="$4"
  # channel env
  local CH=""
  case "$clab" in
    clean)    CH="MERCURY_SFO_GRID_CHAN=0" ;;
    detfloor) CH="MERCURY_SFO_GRID_CHAN=1" ;;
    watt-good)CH="MERCURY_SFO_GRID_CHAN=3 MERCURY_SFO_GRID_WATT_DEPTH_DB=2 MERCURY_SFO_GRID_WATT_FD_HZ=0.5" ;;
    watt-mod) CH="MERCURY_SFO_GRID_CHAN=3 MERCURY_SFO_GRID_WATT_DEPTH_DB=4 MERCURY_SFO_GRID_WATT_FD_HZ=1.0" ;;
    watt-poor)CH="MERCURY_SFO_GRID_CHAN=3 MERCURY_SFO_GRID_WATT_DEPTH_DB=6 MERCURY_SFO_GRID_WATT_FD_HZ=1.5" ;;
  esac
  # arm env
  local AR=""
  case "$arm" in
    # uniform-64 LS-only single-pass (the bare baseline)
    uniform)  AR="MERCURY_SFO_GRID_PCS=0 MERCURY_SFO_GRID_TURBO_ITERS=1 MERCURY_SFO_GRID_NVFIX=0" ;;
    # CFG17 full stack for SIM channels = PAS + TINTERP-seed turbo. The ratio-nvfix
    # is HELD OFF in sim: the in-sim nv never collapses, and on the dispersive
    # det-floor the gate MISFIRES (legit-low nv < measure_var/8 -> over-softens LLRs
    # -> 7/7 -> 0/7). nvfix is a HW-only gated lever (validated via NV_FORCE in
    # --test-cfg17 CELL-C); engaging it in sim ACTIVELY HARMS the dispersive cells.
    stack)    AR="MERCURY_SFO_GRID_PCS=1 MERCURY_SFO_GRID_TURBO_SEED=tinterp MERCURY_SFO_GRID_TURBO_ITERS=4 MERCURY_SFO_GRID_NVFIX=0" ;;
    # diagnostic: the stack WITH nvfix engaged (to quantify the misfire harm)
    stacknvfix) AR="MERCURY_SFO_GRID_PCS=1 MERCURY_SFO_GRID_TURBO_SEED=tinterp MERCURY_SFO_GRID_TURBO_ITERS=4 MERCURY_SFO_GRID_NVFIX=1" ;;
    # diagnostic: PAS single-pass LS (no turbo) — isolates the PAS lever
    pasls)    AR="MERCURY_SFO_GRID_PCS=1 MERCURY_SFO_GRID_TURBO_ITERS=1 MERCURY_SFO_GRID_NVFIX=0" ;;
    genie)    AR="MERCURY_SFO_GRID_PCS=1 MERCURY_SFO_GRID_GENIE=1" ;;
  esac
  local BASE="MERCURY_SFO_GRID=1 MERCURY_SFO_GRID_CODED=1 MERCURY_SFO_GRID_M64=1 MERCURY_SFO_GRID_NSYMB=60"
  local res
  res=$(env $BASE $CH $AR MERCURY_SFO_GRID_ESN0=$esn0 MERCURY_SFO_GRID_SEED=$seed $BIN -m PLOT_PASSBAND -s 16 2>&1 | final)
  [ -z "$res" ] && res="ERR,7"
  echo "${clab},${esn0},${seed},${arm},${res}"
}

CHAN_LABEL="$1"; shift
ESN0S="$1"; shift
SEEDS="$1"; shift
ARMS="${1:-uniform stack genie}"
for e in $ESN0S; do
  for s in $SEEDS; do
    for a in $ARMS; do
      run_cell "$CHAN_LABEL" "$e" "$s" "$a"
    done
  done
done
