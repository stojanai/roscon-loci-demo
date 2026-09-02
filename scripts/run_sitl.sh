#!/bin/bash
# Run ArduCopter SITL with the sprayer enabled (what the audience watches).
# First run compiles the SITL binary (~5 min).
set -e
DEMO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DEMO_ROOT/ardupilot"
source "$DEMO_ROOT/venv/bin/activate"
export PATH="$DEMO_ROOT/ardupilot/Tools/autotest:$PATH"

# --map --console give the on-stage visuals; -w wipes params on first run
sim_vehicle.py -v ArduCopter --console --map \
  --add-param-file="$DEMO_ROOT/scripts/sprayer.parm" "$@"
