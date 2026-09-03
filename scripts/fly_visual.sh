#!/bin/bash
# Launch the SITL drone WITH the moving-map + console GUI at the Kavadarci
# vineyard, mission ready to load. Run this in YOUR OWN terminal (MAVProxy
# needs a real terminal to draw the map) — not from an automated tool.
#
#   ./scripts/fly_visual.sh
#
# Two windows open (on macOS they may be BEHIND the terminal — Cmd+Tab):
#   • Console  — status HUD (mode, alt, GPS, battery)
#   • Map      — satellite-ish map with the drone icon
#
# When the console shows GPS/EKF ready, type these at the MAV> prompt:
#
#   wp load MISSION
#   mode guided
#   arm throttle
#   takeoff 15
#   long DO_SPRAYER 1
#   mode auto
#   graph SERVO_OUTPUT_RAW.servo10_raw     # optional: live pump plot
#
# The drone flies the 7-row coverage over the vineyard; pump (servo10) tracks
# ground speed. 'mode rtl' brings it home. Ctrl-C in the terminal stops it all.
set -e
DEMO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DEMO/ardupilot"
source "$DEMO/venv/bin/activate"
export PATH="$DEMO/ardupilot/Tools/autotest:$PATH"

MISSION="$DEMO/mission/kavadarci_spray.waypoints"
echo "======================================================================"
echo " Kavadarci vineyard spray demo — copy/paste at the MAV> prompt:"
echo
echo "   wp load $MISSION"
echo "   mode guided"
echo "   arm throttle"
echo "   takeoff 15"
echo "   long DO_SPRAYER 1"
echo "   mode auto"
echo "======================================================================"
echo

sim_vehicle.py -v ArduCopter \
  --custom-location=41.440049,21.900371,150,0 \
  --add-param-file="$DEMO/scripts/sprayer.parm" \
  --map --console "$@"
