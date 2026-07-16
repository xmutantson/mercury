#!/bin/bash
# CONNECT-SEED FUSION real-audio A/B — 2 arms on ONE FIX binary via defeat knob (no arm-inversion):
#   FIX  = R + P-alt + connect-fuse   (no env)                        -- the connect-lever candidate
#   BASE = R + P-alt                  (MERCURY_CONNECT_FUSE_DEFEAT=1)  -- the DUTY v2 base
# in-band OFF. secs=280 payload=2e6 start-cfg 100 mode wb traffic random-binary. n=8/arm/band.
# Per band: FIX(cards 0-3, card-base 0) + BASE(cards 4-7, card-base 4) concurrent 16-wide (2 cells/card).
# Bands: WGN:40 / WGN:30 / WGN:25 (profile wgn) + FADE (WGN:30 profile mpm, MODERATE Watterson).
set -u
BIN=/dev/shm/connlever/mercury
H=/dev/shm/dutyv2/harness
OUT=/dev/shm/connlever/ab
N=${N:-8}; SECS=${SECS:-280}
mkdir -p "$OUT/logs"
cd "$H" || exit 3
common="--n $N --bin $BIN --secs $SECS --payload 2000000 --start-cfg 100 --mode wb --traffic random-binary --arm-name all_off"

run_band () {
  local BAND="$1" TAG="$2" PBASE="$3" PROFILE="$4"
  echo "[$(date +%H:%M:%S)] BAND=$BAND profile=$PROFILE : FIX(cards 0-3) + BASE(cards 4-7) concurrent 16-wide"
  nohup python3 parallel_spawner.py $common --cell "$BAND" --profile "$PROFILE" --card-base 0 --port-base $PBASE \
     --tag-prefix ${TAG}fix --out "$OUT/${TAG}fix.json" --logdir "$OUT/logs" > "$OUT/${TAG}fix.spawn.log" 2>&1 &
  nohup python3 parallel_spawner.py $common --cell "$BAND" --profile "$PROFILE" --env MERCURY_CONNECT_FUSE_DEFEAT=1 --card-base 4 --port-base $((PBASE+100)) \
     --tag-prefix ${TAG}bas --out "$OUT/${TAG}bas.json" --logdir "$OUT/logs" > "$OUT/${TAG}bas.spawn.log" 2>&1 &
  while [ ! -f "$OUT/${TAG}fix.json.done" ] || [ ! -f "$OUT/${TAG}bas.json.done" ]; do sleep 10; done
  echo "[$(date +%H:%M:%S)] BAND=$BAND DONE"; touch "$OUT/${TAG}.BANDDONE"
}

run_band "WGN:40" b40 7300 wgn
run_band "WGN:30" b30 7300 wgn
run_band "WGN:25" b25 7300 wgn
run_band "WGN:30" fad 7300 mpm
echo "[$(date +%H:%M:%S)] ALL BANDS DONE"; touch "$OUT/ALLDONE"
