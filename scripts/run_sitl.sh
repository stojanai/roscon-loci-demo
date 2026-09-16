#!/bin/bash
# Run ArduCopter SITL with the sprayer enabled (what the audience watches).
# First run compiles the SITL binary (~5 min).
set -e
DEMO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DEMO_ROOT/ardupilot"
source "$DEMO_ROOT/venv/bin/activate"
export PATH="$DEMO_ROOT/ardupilot/Tools/autotest:$PATH"
# Make scripts/spray_overlay.py importable by MAVProxy's `module load`.
export PYTHONPATH="$DEMO_ROOT/scripts:$PYTHONPATH"

# If a mission was requested: spawn the copter AT the mission's home so the map
# and waypoints line up (otherwise SITL boots at its default home in Australia),
# and upload the mission to the FC once it is connected. MAVProxy's --cmd
# "wp load" draws the pattern on the map immediately but fires before the
# heartbeat, so the background helper does the actual upload to the FC.
# Hardcoded spawn point = mission start (seq 0 of kavadarci_spray.waypoints).
# Format is lat,lon,alt,heading. Edit these if the mission moves.
HOME_LAT=41.4401933
HOME_LON=21.9002405

LOC_ARG=()
if [ -n "$LOCI_MISSION" ] && [ -f "$LOCI_MISSION" ]; then
  LOC_ARG=(--custom-location="$HOME_LAT,$HOME_LON,0,0")
  echo "spawning at mission home: $HOME_LAT,$HOME_LON"
  # The background helper always uploads the mission. By default it then
  # auto-flies hands-free (GUIDED -> arm -> takeoff -> AUTO -> DO_SPRAYER on);
  # autofly.py waits for the copter to be armable on its own. Set NOFLY=1 to
  # upload only and fly by hand from the console. Both python steps run on
  # udpin:14550 sequentially (upload exits before autofly binds), so they never
  # contend for the port.
  if [ -n "$NOFLY" ]; then
    ( python "$DEMO_ROOT/scripts/upload_mission.py" \
        "udpin:127.0.0.1:14550" "$LOCI_MISSION" \
    ) >"$DEMO_ROOT/mission/last_upload.log" 2>&1 &
    cat <<'EOF'
--------------------------------------------------------------------
NOFLY: the helper only UPLOADS the mission. You fly by hand.
Wait until mission/last_upload.log ends with "uploaded and VERIFIED"
(or `wp list` shows 32), then in the MAVProxy console EITHER run the script:
    script scripts/sitl_sprayer.scr    # guided/arm/takeoff/auto; mission sprays rows
OR type the steps yourself:
    wp list
    mode guided
    arm throttle
    takeoff 15
    mode auto
    long DO_SPRAYER 0 0 0 0 0 0 0      # pump off; mission enables it per row
Verify the pump: `graph SERVO_OUTPUT_RAW.servo10_raw` rises above ~1000 only
while the copter is flying a crop row (and moving faster than SPRAY_SPEED_MIN),
and drops back to ~1000 on the ferry legs and headland turns.
--------------------------------------------------------------------
EOF
  else
    ( python "$DEMO_ROOT/scripts/upload_mission.py" \
          "udpin:127.0.0.1:14550" "$LOCI_MISSION" \
        && python "$DEMO_ROOT/scripts/autofly.py" \
          "udpin:127.0.0.1:14550" 15 \
      ) >"$DEMO_ROOT/mission/last_upload.log" 2>&1 &
    cat <<'EOF'
--------------------------------------------------------------------
Hands-free: the background helper uploads the mission, waits until the copter
is armable, then flies it (GUIDED -> arm -> takeoff -> AUTO). The mission
sprays each crop row on its own (ferry legs and headland turns stay dry).
No typing needed on stage.

Follow along in mission/last_upload.log — it ends with:
    "... uploaded and VERIFIED"
    "autofly: AUTO engaged — mission is flying"
    "autofly: DO_SPRAYER=0 sent — pump off; mission sprays each row"

To fly by hand instead, re-run with:  NOFLY=1 ./scripts/demo.sh sitl
--------------------------------------------------------------------
EOF
  fi
fi

# --map --console give the on-stage visuals; -w wipes params on first run.
# -m "--load-module spray_overlay" paints the sprayed swath yellow on the map,
# with width scaled to pump throughput (0 -> 20 m).
# (camera_spray.py is the experimental green-only "see and spray" module; it is
# kept in scripts/ but NOT loaded by default because it needs the flight path to
# cross green and its threshold tuned. Load it by hand with `module load
# camera_spray` when you want to demo green detection.)
sim_vehicle.py -v ArduCopter --console --map "${LOC_ARG[@]}" \
  -m "--load-module spray_overlay" \
  --add-param-file="$DEMO_ROOT/scripts/sprayer.parm" "$@"
