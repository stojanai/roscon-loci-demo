#!/bin/bash
# Educational LOCI demo runner — keep the tree clean, apply one teaching case as
# code, rebuild, and fly. Purpose: show engineers to track performance metrics
# (timing / stack / memory) from the compiled artifact, not by eyeballing source.
#
#   ./scripts/demo.sh clean          # restore ArduPilot to a clean baseline
#   ./scripts/demo.sh case <1-4>     # apply a teaching case AS CODE, then build
#   ./scripts/demo.sh build          # build the CubeOrange ELF from current tree
#   ./scripts/demo.sh sitl           # launch SITL and auto-load the saved mission
#   ./scripts/demo.sh status         # show what's applied
#
# Cases:  1 = median speed filter (stack)   2 = wind-drift comp (timing)
#         3 = ROS velocity smoothing (timing/off-track)   4 = IR camera task
set -e
DEMO="$(cd "$(dirname "$0")/.." && pwd)"
AP="$DEMO/ardupilot"
export PATH="$DEMO/toolchain/bin:$PATH"
CAM="ArduCopter/vision_ir.cpp ArduCopter/vision_ir.h"
MISSION="$DEMO/mission/kavadarci_spray.waypoints"

clean() {
  cd "$AP"
  git checkout -- libraries/AC_Sprayer ArduCopter/mode_guided.cpp \
      ArduCopter/Copter.cpp ArduCopter/Copter.h 2>/dev/null || true
  rm -f $CAM
  echo "tree clean (baseline)."
}

apply_case() {
  clean; cd "$AP"
  case "$1" in
    1) git apply "$DEMO/patches/speed-smoothing-bug.patch" ;;
    2) git apply "$DEMO/patches/drift-compensation-bug.patch" ;;
    3) git apply "$DEMO/patches/velocity-smoothing.patch" ;;
    4) git apply "$DEMO/patches/vision-ir-camera.patch" ;;
    *) echo "usage: demo.sh case <1|2|3|4>"; exit 1 ;;
  esac
  echo "applied case $1 as code. Now: ./scripts/demo.sh build   (or measure with LOCI in the ardupilot/ session)"
}

build() {
  cd "$AP"; source "$DEMO/venv/bin/activate"
  ./waf configure --board CubeOrange >/dev/null
  ./waf copter
  cp build/CubeOrange/bin/arducopter "$DEMO/artifacts/arducopter-current.elf"
  echo "ELF for LOCI -> $DEMO/artifacts/arducopter-current.elf"
  arm-none-eabi-size "$DEMO/artifacts/arducopter-current.elf"
}

case "${1:-}" in
  clean)  clean ;;
  case)   apply_case "$2" ;;
  build)  build ;;
  sitl)   [ -f "$MISSION" ] || { echo "mission not found: $MISSION"; exit 1; }
          echo "auto-loading mission: $MISSION"
          # LOCI_MISSION drives run_sitl: it spawns the copter at the mission's
          # home and runs the single background uploader (which waits for the FC
          # to be ready before pushing the mission). No MAVProxy `wp load` here —
          # a second uploader collides with this one during the transfer.
          export LOCI_MISSION="$MISSION"
          exec "$DEMO/scripts/run_sitl.sh" "${@:2}" ;;
  status) cd "$AP"; git status --short; ls "$AP/ArduCopter/vision_ir.cpp" 2>/dev/null && echo "(case 4 camera present)" ;;
  *) echo "usage: demo.sh {clean|case <n>|build|sitl|status}"; exit 1 ;;
esac
