'''
MAVProxy map overlay: paint the sprayed swath in YELLOW.

Loaded inside MAVProxy (the map lives in that process), it watches the pump
servo output and the vehicle position, and drops filled yellow circles along
the flight path wherever the sprayer is actually delivering.

The swath WIDTH tracks nozzle throughput: ~0 m when the pump is off, up to
`maxwidth` metres (default 20) at full pump output. Throughput is read from the
pump servo PWM (SERVO10, function 22 = sprayer pump) scaled between its MIN/MAX.

Load it (run_sitl.sh does this automatically):
    module load spray_overlay
Commands:
    spray status      show current settings / throughput
    spray clear       wipe the yellow overlay
    spray set maxwidth 20
'''
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.mavproxy_map import mp_slipmap

LAYER = 'SprayTrail'
HUD_LAYER = 'SprayHUD'
YELLOW = (255, 255, 0)  # RGB, as MAVProxy's slipmap expects
WHITE = (255, 255, 255)


class SprayOverlayModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SprayOverlayModule, self).__init__(
            mpstate, "spray", "yellow sprayed-area map overlay")
        self.spray_settings = mp_settings.MPSettings([
            ('maxwidth', float, 5.0),   # swath width (m) at full throughput
            ('pwm_min', int, 1000),      # pump PWM at zero throughput
            ('pwm_max', int, 2000),      # pump PWM at full throughput
            ('deadband', float, 0.03),   # ignore throughput below this fraction
            ('rate_hz', float, 5.0),     # requested SERVO_OUTPUT_RAW rate
            # --- on-map readout (speed + delivered dose) ---
            ('hud', bool, True),         # draw the speed / L-per-ha label on the map
            ('hud_size', float, 0.6),    # label text size
            ('swath_m', float, 5.0),     # CONSTANT physical nozzle swath (m) for dose
            ('pump_lps', float, 0.075),  # full-pump flow (l/s); mirrors SITL pump_max_rate
        ])
        self.add_command('spray', self.cmd_spray, 'sprayed-area overlay',
                         ['status', 'clear', 'set (SPRAYSETTING)'])
        self.pump_pwm = 0
        self.have_pump = False
        self.last_pos = None          # (lat, lon) of the last circle we drew
        self.count = 0
        self.requested = 0            # last time we asked for the servo stream
        self.groundspeed = 0.0        # m/s, from GLOBAL_POSITION_INT vx/vy

    # ---- throughput ------------------------------------------------------
    def throughput_fraction(self):
        '''0.0 (pump off) .. 1.0 (full) from the pump servo PWM.'''
        lo = self.spray_settings.pwm_min
        hi = self.spray_settings.pwm_max
        if not self.have_pump or hi <= lo:
            return 0.0
        f = (self.pump_pwm - lo) / float(hi - lo)
        return max(0.0, min(1.0, f))

    def liters_per_ha(self):
        '''instantaneous delivered dose (L/ha) from actual flow and speed.

        flow (l/s) = throughput_fraction * pump_lps; over a CONSTANT swath the
        drone treats (swath_m * groundspeed) m^2/s, so
            dose = flow / (swath_m * groundspeed)  [L/m^2]  -> * 1e4 = L/ha.
        The groundspeed cancels (pump flow tracks speed), so this holds ~steady
        while spraying even as the copter slows at turns.
        '''
        gs = self.groundspeed
        w = self.spray_settings.swath_m
        if gs <= 0.2 or w <= 0.0:
            return 0.0
        flow_lps = self.throughput_fraction() * self.spray_settings.pump_lps
        return (flow_lps / (w * gs)) * 1.0e4

    # ---- MAVLink ---------------------------------------------------------
    def request_servo_stream(self):
        '''Ask the FC to stream SERVO_OUTPUT_RAW so we can read the pump.'''
        try:
            import pymavlink.mavutil as mavutil
            interval_us = int(1e6 / max(1.0, self.spray_settings.rate_hz))
            self.master.mav.command_long_send(
                self.settings.target_system, self.settings.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
                interval_us, 0, 0, 0, 0, 0)
        except Exception:
            pass

    def idle_task(self):
        # (Re)request the pump stream until it actually arrives.
        if not self.have_pump:
            now = time.time()
            if now - self.requested > 3.0:
                self.requested = now
                self.request_servo_stream()

    def mavlink_packet(self, m):
        t = m.get_type()
        if t == 'SERVO_OUTPUT_RAW':
            self.pump_pwm = m.servo10_raw
            self.have_pump = True
        elif t == 'GLOBAL_POSITION_INT':
            self.groundspeed = ((m.vx * m.vx + m.vy * m.vy) ** 0.5) / 100.0
            lat, lon = m.lat * 1e-7, m.lon * 1e-7
            self.maybe_mark(lat, lon)
            self.update_hud(lat, lon)

    # ---- drawing ---------------------------------------------------------
    def maybe_mark(self, lat, lon):
        frac = self.throughput_fraction()
        if frac <= self.spray_settings.deadband:
            return
        width = frac * self.spray_settings.maxwidth
        radius = width / 2.0
        if radius <= 0.1:
            return
        # Keep circles overlapping into a continuous band without spamming
        # thousands of objects: only add one once we've moved ~60% of a radius.
        if self.last_pos is not None:
            moved = mp_util.gps_distance(self.last_pos[0], self.last_pos[1],
                                         lat, lon)
            if moved < max(0.5, radius * 0.6):
                return
        slipmap = getattr(self.mpstate, 'map', None)
        if slipmap is None:
            return
        self.count += 1
        slipmap.add_object(mp_slipmap.SlipCircle(
            'spray%u' % self.count, LAYER, (lat, lon),
            radius, YELLOW, linewidth=-1))  # linewidth -1 = filled
        self.last_pos = (lat, lon)

    def update_hud(self, lat, lon):
        '''draw a live "groundspeed | L/ha" label on the map at the copter.'''
        if not self.spray_settings.hud:
            return
        slipmap = getattr(self.mpstate, 'map', None)
        if slipmap is None:
            return
        text = "%.1f m/s   %.1f L/ha" % (self.groundspeed, self.liters_per_ha())
        # offset the label ~6 m NE of the copter so it doesn't sit under the icon
        plat = lat + 6.0 / 111320.0
        try:
            slipmap.add_object(mp_slipmap.SlipLabel(
                'spray_hud', (plat, lon), text, HUD_LAYER, WHITE,
                self.spray_settings.hud_size))
        except Exception:
            pass

    def clear(self):
        slipmap = getattr(self.mpstate, 'map', None)
        if slipmap is not None:
            slipmap.add_object(mp_slipmap.SlipClearLayer(LAYER))
            slipmap.add_object(mp_slipmap.SlipClearLayer(HUD_LAYER))
        self.count = 0
        self.last_pos = None

    # ---- commands --------------------------------------------------------
    def cmd_spray(self, args):
        if not args or args[0] == 'status':
            print("spray: throughput %.0f%% (pump pwm %u), %.1f m/s, %.1f L/ha, "
                  "width now %.1f m, maxwidth %.1f m, marks %u"
                  % (self.throughput_fraction() * 100.0, self.pump_pwm,
                     self.groundspeed, self.liters_per_ha(),
                     self.throughput_fraction() * self.spray_settings.maxwidth,
                     self.spray_settings.maxwidth, self.count))
        elif args[0] == 'clear':
            self.clear()
            print("spray: overlay cleared")
        elif args[0] == 'set':
            self.spray_settings.command(args[1:])
        else:
            print("usage: spray <status|clear|set>")


def init(mpstate):
    '''initialise module'''
    return SprayOverlayModule(mpstate)
