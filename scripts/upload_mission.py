#!/usr/bin/env python3
# Upload a QGC WPL mission to the flight controller AFTER it is truly ready.
#
# Three things bite a naive uploader against a sim_vehicle MAVProxy:
#   1. The first heartbeat arrives before the mission subsystem is initialised,
#      so an early MISSION_COUNT is rejected with NO_SPACE ("Only 0 items are
#      supported"). We wait for a readiness signal first.
#   2. At boot MAVProxy pulls all ~1400 params over FTP and the --map module
#      auto-downloads the mission to draw it. Both saturate / contend for the
#      single MAVLink link; pushing a mission into that storm gets item requests
#      starved and the FC cancels the transfer (OPERATION_CANCELLED / "Mission
#      upload timeout"). So we let the link go quiet (SETTLE_S) before pushing.
#   3. Modern ArduPilot wants MISSION_ITEM_INT, not the float MISSION_ITEM
#      (it warns "GCS should send MISSION_ITEM_INT"). We send INT.
#
# The FC's MISSION_ACK == ACCEPTED is authoritative — it is sent only after the
# FC has received and validated every item. We trust it and do NOT issue a
# read-back mission_request_list, which would race the map module's download.
#
#   upload_mission.py <connection> <mission.waypoints>
#   e.g. upload_mission.py udpin:127.0.0.1:14550 mission/kavadarci_spray.waypoints
import sys
import time
from pymavlink import mavutil, mavwp

REQ = ["MISSION_REQUEST", "MISSION_REQUEST_INT"]
SETTLE_S = 12  # let MAVProxy's boot param-FTP + map mission fetch finish
MISSION_TYPE = getattr(mavutil.mavlink, "MAV_MISSION_TYPE_MISSION", 0)


def main():
    if len(sys.argv) != 3:
        print("usage: upload_mission.py <connection> <mission.waypoints>")
        return 2
    conn_str, wpfile = sys.argv[1], sys.argv[2]

    wl = mavwp.MAVWPLoader()
    wl.load(wpfile)
    count = wl.count()
    print("upload_mission: parsed %u waypoints from %s" % (count, wpfile))

    m = mavutil.mavlink_connection(conn_str)
    print("upload_mission: waiting for heartbeat on %s ..." % conn_str)
    m.wait_heartbeat()
    print("upload_mission: heartbeat from system %u component %u"
          % (m.target_system, m.target_component))

    # Give the FC a chance to print "Ready" first, then let the boot-time link
    # storm (param FTP + map mission fetch) drain before we push anything.
    wait_ready_hint(m)
    print("upload_mission: letting the link settle for %us before pushing"
          % SETTLE_S)
    drain(m, SETTLE_S)

    # Push the mission. A bare MISSION_COUNT already replaces any existing
    # mission, so we do NOT send MISSION_CLEAR_ALL first — its separate
    # ACCEPTED ack was being mistaken for a successful upload, leaving the FC
    # with 0 items. Early in boot the subsystem reports zero capacity and
    # rejects with NO_SPACE, so retry until the FC ACCEPTS *and* a read-back
    # confirms the count. The read-back is the real safety net: the ACCEPT ack
    # alone can be an echo of an earlier exchange.
    deadline = time.time() + 90
    attempt = 0
    while time.time() < deadline:
        attempt += 1
        send_count(m, count)
        ok, why = run_transfer(m, wl, count)
        if ok and readback_ok(m, count):
            print("upload_mission: %u waypoints uploaded and VERIFIED" % count)
            return 0
        print("upload_mission: attempt %u not confirmed (%s), retrying"
              % (attempt, why))
        time.sleep(2)

    print("upload_mission: FAILED to upload within timeout")
    return 1


def send_count(m, count):
    """Announce the item count, passing mission_type where the dialect has it."""
    try:
        m.mav.mission_count_send(m.target_system, m.target_component,
                                 count, MISSION_TYPE)
    except TypeError:  # older pymavlink without mission_type arg
        m.mav.mission_count_send(m.target_system, m.target_component, count)


def readback_ok(m, expected, tries=4):
    """Confirm the FC now holds exactly `expected` items (authoritative check)."""
    for _ in range(tries):
        try:
            m.mav.mission_request_list_send(
                m.target_system, m.target_component, MISSION_TYPE)
        except TypeError:
            m.mav.mission_request_list_send(m.target_system, m.target_component)
        msg = m.recv_match(type="MISSION_COUNT", blocking=True, timeout=3)
        if msg is None:
            continue
        if msg.count == expected:
            return True
        print("upload_mission: read-back count %u != %u" % (msg.count, expected))
        return False
    print("upload_mission: read-back got no MISSION_COUNT")
    return False


def drain(m, seconds):
    """Consume and discard incoming traffic for `seconds` to let the link idle."""
    end = time.time() + seconds
    while time.time() < end:
        m.recv_match(blocking=True, timeout=0.5)


def wait_ready_hint(m, timeout=30):
    """Best-effort wait for the FC's "Ready" STATUSTEXT before first attempt."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
        if msg and "Ready" in (msg.text or ""):
            print("upload_mission: FC reports '%s'" % msg.text)
            return
    print("upload_mission: no 'Ready' seen yet, proceeding with retries")


def run_transfer(m, wl, count):
    deadline = time.time() + 15
    highest = -1  # highest item sequence we have actually served
    while time.time() < deadline:
        msg = m.recv_match(type=REQ + ["MISSION_ACK"], blocking=True, timeout=5)
        if msg is None:
            return False, "timeout"
        t = msg.get_type()
        if t == "MISSION_ACK":
            code = getattr(msg, "type", -1)
            ack = mavutil.mavlink.enums["MAV_MISSION_RESULT"].get(code)
            name = ack.name if ack else str(code)
            if code != 0:
                return False, name          # NO_SPACE / INVALID etc. -> retry
            if highest >= count - 1:
                return True, name           # genuine end-of-transfer accept
            continue                        # stray ack (e.g. clear echo) — ignore
        seq = msg.seq
        if seq >= count:
            continue
        send_item_int(m, wl.wp(seq))
        if seq > highest:
            highest = seq
    return False, "no ACK"


def send_item_int(m, wp):
    """Send one waypoint as MISSION_ITEM_INT (lat/lon scaled to 1e7 degrees)."""
    m.mav.mission_item_int_send(
        m.target_system, m.target_component,
        wp.seq, wp.frame, wp.command, wp.current, wp.autocontinue,
        wp.param1, wp.param2, wp.param3, wp.param4,
        int(round(wp.x * 1e7)), int(round(wp.y * 1e7)), wp.z,
        MISSION_TYPE)


if __name__ == "__main__":
    sys.exit(main())
