#!/usr/bin/env python3
"""Fly the Kavadarci vineyard spray mission on a loop until stopped (Ctrl-C).

Connects to a running ArduCopter SITL, uploads mission/kavadarci_spray.waypoints,
then repeatedly: takeoff -> sprayer ON -> AUTO coverage -> RTL/land -> repeat.
Streams ground speed + pump PWM the whole time.
"""
import sys, time, signal, os
from pymavlink import mavutil, mavwp

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WP_FILE = os.path.join(HERE, "mission", "kavadarci_spray.waypoints")
CONN = os.environ.get("FLY_CONN", "tcp:127.0.0.1:5760")
ALT = 15.0

stop = False
signal.signal(signal.SIGINT,  lambda *a: globals().__setitem__("stop", True))
signal.signal(signal.SIGTERM, lambda *a: globals().__setitem__("stop", True))

def log(m): print(m, flush=True)

def cmd(m, c, *p):
    p = list(p) + [0]*(7-len(p))
    m.mav.command_long_send(m.target_system, m.target_component, c, 0, *p)
    a = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=4)
    return a.result if a else None

def upload_mission(m):
    wl = mavwp.MAVWPLoader()
    wl.load(WP_FILE)
    m.waypoint_clear_all_send()
    m.mav.mission_count_send(m.target_system, m.target_component, wl.count())
    for _ in range(wl.count()+2):
        msg = m.recv_match(type=["MISSION_REQUEST","MISSION_REQUEST_INT","MISSION_ACK"],
                           blocking=True, timeout=5)
        if not msg: break
        if msg.get_type()=="MISSION_ACK": break
        m.mav.send(wl.wp(msg.seq))
        if msg.seq == wl.count()-1:
            m.recv_match(type="MISSION_ACK", blocking=True, timeout=5); break
    log(f"mission uploaded: {wl.count()} items")

def request_streams(m):
    m.mav.request_data_stream_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

def wait_ready(m):
    log("waiting for GPS fix + EKF ...")
    request_streams(m)
    t0=time.time(); fix=False
    while time.time()-t0 < 60 and not stop:
        x=m.recv_match(type=["GPS_RAW_INT","STATUSTEXT"], blocking=True, timeout=2)
        if not x: continue
        if x.get_type()=="STATUSTEXT" and ("EKF" in x.text or "GPS" in x.text):
            log("  "+x.text)
        elif x.get_type()=="GPS_RAW_INT" and x.fix_type>=3:
            fix=True; break
    log("GPS fix acquired" if fix else "proceeding without confirmed fix")
    time.sleep(3)

def alt_now(m):
    p=m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    return p.relative_alt/1000.0 if p else -1.0

def fly_once(m, run):
    log(f"\n=== spray run #{run} ===")
    cmd(m, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 4)          # GUIDED
    armed=False
    for _ in range(20):
        if stop: return
        r=cmd(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        if r==0: armed=True; log("armed"); break
        s=m.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
        if s and ("PreArm" in s.text or "Arm" in s.text): log("  "+s.text)
        time.sleep(2)
    if not armed:
        log("arm failed (see PreArm messages above)"); return
    cmd(m, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,ALT)
    t0=time.time()
    while not stop and time.time()-t0<30:
        if alt_now(m) >= ALT-2: break
        time.sleep(0.5)
    log("at altitude - sprayer ON, switching to AUTO")
    cmd(m, mavutil.mavlink.MAV_CMD_DO_SPRAYER, 1)
    # switch to AUTO and confirm the mode actually took, then start the mission
    for _ in range(8):
        cmd(m, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 3)      # AUTO=3
        hb=m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and (hb.custom_mode==3): log("AUTO engaged"); break
        time.sleep(1)
    cmd(m, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0)
    m.mav.set_mode_send(m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3)
    # monitor until disarmed (mission done + RTL land) or stop
    gs=0; last=0; t0=time.time()
    while not stop:
        x=m.recv_match(type=["VFR_HUD","SERVO_OUTPUT_RAW","HEARTBEAT"],blocking=True,timeout=2)
        if not x: continue
        t=x.get_type()
        if t=="VFR_HUD": gs=x.groundspeed
        elif t=="SERVO_OUTPUT_RAW" and time.time()-last>2:
            last=time.time()
            log(f"  t{time.time()-t0:4.0f}s  gs={gs:4.1f} m/s  pump={x.servo10_raw}  spin={x.servo11_raw}")
        elif t=="HEARTBEAT":
            armed = x.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if not armed and time.time()-t0>30:
                log("landed + disarmed - run complete"); return
        if time.time()-t0>400: log("timeout - forcing RTL"); cmd(m,20); return

def main():
    log(f"connecting {CONN} ...")
    m=mavutil.mavlink_connection(CONN)
    m.wait_heartbeat(timeout=30)
    log("linked to vehicle")
    wait_ready(m)
    upload_mission(m)
    fly_once(m, 1)          # fly the coverage once (the flight shown in spray_map.html)
    log("\nmission complete - stopped.")

if __name__=="__main__":
    main()
