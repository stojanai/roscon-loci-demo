#!/usr/bin/env python3
# Hands-free auto-fly for the spray demo.
#
# After the mission is on the FC, wait until the copter is genuinely armable
# (EKF/GPS ready, pre-arm passing), then fly it with no typing on stage:
#
#     GUIDED -> arm -> takeoff -> AUTO -> DO_SPRAYER off
#
# The mission itself toggles the sprayer per crop row (a DO_SPRAYER 1/0 pair
# around each N-S survey line), so the ferry legs and headland turns stay dry.
# We send an explicit DO_SPRAYER=0 right after AUTO engages to guarantee the
# pump starts OFF (in case a previous run left it enabled); the mission then
# turns it on only over each row. The pump physically runs once the copter is
# spraying a row and groundspeed passes SPRAY_SPEED_MIN.
#
#   autofly.py <connection> [takeoff_alt_m]
#   e.g. autofly.py udpin:127.0.0.1:14550 15
import sys
import time
from pymavlink import mavutil


def main():
    conn = sys.argv[1] if len(sys.argv) > 1 else "udpin:127.0.0.1:14550"
    alt = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

    m = mavutil.mavlink_connection(conn)
    print("autofly: waiting for heartbeat on %s ..." % conn)
    m.wait_heartbeat()
    print("autofly: heartbeat from system %u component %u"
          % (m.target_system, m.target_component))

    if not set_mode(m, "GUIDED"):
        print("autofly: could not enter GUIDED; aborting (fly manually)")
        return 1

    # arm() retries, which naturally waits out pre-arm (GPS/EKF settling).
    if not arm(m):
        print("autofly: never became armable; aborting (fly manually)")
        return 1

    print("autofly: taking off to %.0f m" % alt)
    send_cmd(m, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=alt)
    if not wait_alt(m, alt * 0.95):
        print("autofly: takeoff altitude not confirmed; continuing anyway")

    if not set_mode(m, "AUTO"):
        print("autofly: could not enter AUTO; aborting")
        return 1
    print("autofly: AUTO engaged — mission is flying")

    # Force the sprayer OFF at the start; the mission enables it per crop row.
    send_cmd(m, mavutil.mavlink.MAV_CMD_DO_SPRAYER, p1=0)
    print("autofly: DO_SPRAYER=0 sent — pump off; mission sprays each row")
    return 0


def send_cmd(m, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    m.mav.command_long_send(m.target_system, m.target_component,
                            command, 0, p1, p2, p3, p4, p5, p6, p7)


def set_mode(m, name, timeout=15):
    """Set flight mode by name and confirm the FC switched, with retries."""
    mapping = m.mode_mapping() or {}
    if name not in mapping:
        print("autofly: mode %s unknown to this vehicle" % name)
        return False
    want = mapping[name]
    deadline = time.time() + timeout
    while time.time() < deadline:
        m.set_mode(want)
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if msg and msg.custom_mode == want:
            print("autofly: mode %s" % name)
            return True
    return False


def arm(m, timeout=90):
    """Send arm and wait until motors report armed, retrying through pre-arm."""
    deadline = time.time() + timeout
    announced = 0
    while time.time() < deadline:
        send_cmd(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1)
        end = time.time() + 3
        while time.time() < end:
            msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg and (msg.base_mode
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("autofly: armed")
                return True
        if time.time() - announced > 8:
            print("autofly: waiting to become armable (pre-arm / GPS) ...")
            announced = time.time()
    return False


def wait_alt(m, target, timeout=30):
    """Wait until relative altitude reaches `target` metres."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type="GLOBAL_POSITION_INT",
                           blocking=True, timeout=2)
        if msg and msg.relative_alt / 1000.0 >= target:
            print("autofly: reached %.1f m" % (msg.relative_alt / 1000.0))
            return True
    return False


if __name__ == "__main__":
    sys.exit(main())
