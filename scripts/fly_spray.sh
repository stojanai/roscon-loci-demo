#!/bin/bash
# Start ArduCopter SITL at the Kavadarci vineyard and fly the spray coverage
# mission once (the flight shown in mission/spray_map.html), then stop.
#
#   ./scripts/fly_spray.sh              # start sim + fly once + stop
#   ./scripts/fly_spray.sh --keep       # leave the sim running afterwards
set -e
DEMO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DEMO"
source venv/bin/activate
HOME_LL="41.440049,21.900371,150,0"   # Kavadarci / Tikves
STARTED=""

if ! pgrep -f "build/sitl/bin/arducopter" >/dev/null; then
  echo "starting SITL at $HOME_LL ..."
  ( cd ardupilot && ./build/sitl/bin/arducopter --model + --speedup 1 \
      --defaults Tools/autotest/default_params/copter.parm,../scripts/sprayer.parm \
      --home "$HOME_LL" -I0 >/tmp/fly_sitl.log 2>&1 ) &
  STARTED=1
  sleep 8
else
  echo "SITL already running - reusing it"
fi

cleanup(){ [ -n "$STARTED" ] && [ "$1" != "--keep" ] && pkill -f "build/sitl/bin/arducopter" 2>/dev/null || true; }
trap 'cleanup "$1"' EXIT

PYTHONUNBUFFERED=1 python scripts/fly_spray.py
[ "$1" = "--keep" ] && { echo "SITL left running (--keep)"; trap - EXIT; }
