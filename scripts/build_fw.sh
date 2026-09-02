#!/bin/bash
# Build the CubeOrange (STM32H7) ArduCopter ELF — the artifact LOCI analyzes.
# Usage: ./build_fw.sh baseline   -> clean AC_Sprayer, build (the "good" firmware)
#        ./build_fw.sh case1       -> apply stack/median-filter bug, build
#        ./build_fw.sh case2       -> apply drift-compensation timing bug, build
#
# NOTE: never run git operations on the tree while a build is in flight — waf
# reads the working copy as it compiles, and a mid-build `git stash`/`checkout`
# silently produces a firmware that doesn't match the source.
set -e
DEMO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DEMO_ROOT/ardupilot"
source "$DEMO_ROOT/venv/bin/activate"
export PATH="$DEMO_ROOT/toolchain/bin:$PATH"

SPRAYER=libraries/AC_Sprayer
git checkout -- "$SPRAYER"     # always start from a clean baseline
case "${1:-baseline}" in
  baseline) OUT=baseline ;;
  case1)    git apply "$DEMO_ROOT/patches/speed-smoothing-bug.patch";   OUT=bugged ;;
  case2)    git apply "$DEMO_ROOT/patches/drift-compensation-bug.patch"; OUT=case2-drift ;;
  *) echo "usage: $0 [baseline|case1|case2]"; exit 1 ;;
esac

./waf configure --board CubeOrange >/dev/null
touch "$SPRAYER"/AC_Sprayer.cpp "$SPRAYER"/AC_Sprayer.h   # force recompile
./waf copter
cp build/CubeOrange/bin/arducopter "$DEMO_ROOT/artifacts/arducopter-$OUT.elf"
git checkout -- "$SPRAYER"     # leave tree clean
echo
echo "ELF for LOCI: $DEMO_ROOT/artifacts/arducopter-$OUT.elf"
arm-none-eabi-size "$DEMO_ROOT/artifacts/arducopter-$OUT.elf"
