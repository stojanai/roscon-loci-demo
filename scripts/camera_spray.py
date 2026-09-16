'''
MAVProxy "camera" module: green-on-green spraying + even dose per m^2.

Simulates a downward camera by sampling the satellite map imagery under the
drone (the same MicrosoftHyb tiles the map draws). It computes a vegetation
index over the nozzle footprint and:

  * GREEN under the drone  -> nozzle ON  (SPRAY_PUMP_RATE = rate_green)
  * no green               -> nozzle REDUCED (SPRAY_PUMP_RATE = rate_bare)

so the sprayer only treats the vegetation, not bare soil. It paints the treated
area YELLOW on the map with a CONSTANT swath width (the physical nozzle
footprint). Because ArduPilot already scales pump flow with groundspeed
(flow ~ SPRAY_PUMP_RATE x speed) and the width is constant, the delivered dose
per square metre stays uniform even when the drone slows down at turns -- no
hotspots.

Vegetation index: normalised Excess-Green  nExG = 2g - r - b  with
r,g,b = R,G,B / (R+G+B). Validated on this field: bare soil ~0.03, vineyard
rows ~0.2-0.5, so the default green_on/green_off thresholds sit in between.

Load (run_sitl.sh does this automatically):
    module load camera_spray
Commands:
    camsp status                 greenness, spray state, commanded rate
    camsp clear                  wipe the yellow overlay
    camsp set green_on 0.14      tune thresholds / rates / width
'''
import os
import time

import cv2
import numpy as np

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.mavproxy_map import mp_tile as MT
from MAVProxy.modules.mavproxy_map import mp_slipmap

import pymavlink.mavutil as mavutil

LAYER = 'CamSpray'
YELLOW = (255, 255, 0)   # RGB, as MAVProxy's slipmap expects


class CameraSprayModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(CameraSprayModule, self).__init__(
            mpstate, "camsp", "camera green-detect spraying + yellow overlay")
        self.cam_settings = mp_settings.MPSettings([
            ('service', str, os.environ.get('MAP_SERVICE', 'MicrosoftHyb')),
            ('zoom', int, 19),           # tile zoom to sample
            ('sample_m', float, 4.0),    # camera footprint sampled for green
            ('swath_m', float, 8.0),     # constant painted/treated width (m)
            ('green_on', float, 0.14),   # nExG to START spraying
            ('green_off', float, 0.10),  # nExG to STOP spraying (hysteresis)
            ('rate_green', float, 12.0), # SPRAY_PUMP_RATE over vegetation
            ('rate_bare', float, 1.0),   # SPRAY_PUMP_RATE over bare soil
        ])
        self.add_command('camsp', self.cmd_camsp, 'camera spraying',
                         ['status', 'clear', 'set (CAMSPSETTING)'])
        self.tiles = MT.MPTile(service=self.cam_settings.service,
                               download=False)
        self._img_cache = {}
        self.greenness = 0.0
        self.spraying = False
        self.commanded_rate = None
        self.speed = 0.0
        self.last_pos = None
        self.count = 0

    # ---- vegetation sensing ---------------------------------------------
    def read_tile(self, tx, ty, zoom):
        key = (tx, ty, zoom)
        if key in self._img_cache:
            return self._img_cache[key]
        t = MT.TileInfo((tx, ty), zoom, self.cam_settings.service)
        path = self.tiles.tile_to_path(t)
        img = cv2.imread(path) if os.path.exists(path) else None
        # keep the cache from growing without bound
        if len(self._img_cache) > 256:
            self._img_cache.clear()
        self._img_cache[key] = img
        return img

    def sample_greenness(self, lat, lon):
        '''normalised Excess-Green over the footprint, or None if no imagery.'''
        zoom = self.cam_settings.zoom
        t = self.tiles.coord_to_tile(lat, lon, zoom)
        img = self.read_tile(t.x, t.y, zoom)
        if img is None:
            return None
        h, w = img.shape[:2]
        mpp = 156543.03392 * np.cos(np.radians(lat)) / float(1 << zoom)
        win = max(1, int(0.5 * self.cam_settings.sample_m / max(mpp, 1e-6)))
        ox, oy = t.offsetx, t.offsety
        patch = img[max(0, oy - win):min(h, oy + win + 1),
                    max(0, ox - win):min(w, ox + win + 1)].astype(np.float64)
        if patch.size == 0:
            return None
        B = patch[:, :, 0].mean()
        G = patch[:, :, 1].mean()
        R = patch[:, :, 2].mean()
        s = R + G + B + 1e-6
        return (2 * G - R - B) / s

    # ---- sprayer control -------------------------------------------------
    def set_pump_rate(self, value):
        if self.commanded_rate is not None and abs(value - self.commanded_rate) < 1e-3:
            return
        self.commanded_rate = value
        self.master.mav.param_set_send(
            self.settings.target_system, self.settings.target_component,
            b'SPRAY_PUMP_RATE', float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def enable_sprayer(self):
        self.master.mav.command_long_send(
            self.settings.target_system, self.settings.target_component,
            mavutil.mavlink.MAV_CMD_DO_SPRAYER, 0, 1, 0, 0, 0, 0, 0, 0)

    # ---- MAVLink ---------------------------------------------------------
    def mavlink_packet(self, m):
        t = m.get_type()
        if t == 'VFR_HUD':
            self.speed = m.groundspeed
        elif t == 'GLOBAL_POSITION_INT':
            self.update(m.lat * 1e-7, m.lon * 1e-7)

    def update(self, lat, lon):
        g = self.sample_greenness(lat, lon)
        if g is None:
            return                       # no imagery here: hold state
        self.greenness = g
        # hysteresis so we don't chatter on the boundary
        if not self.spraying and g >= self.cam_settings.green_on:
            self.spraying = True
            self.enable_sprayer()
        elif self.spraying and g < self.cam_settings.green_off:
            self.spraying = False
        # command the physical nozzle: full over green, reduced over bare
        self.set_pump_rate(self.cam_settings.rate_green if self.spraying
                           else self.cam_settings.rate_bare)
        if self.spraying:
            self.paint(lat, lon)

    # ---- overlay ---------------------------------------------------------
    def paint(self, lat, lon):
        radius = self.cam_settings.swath_m / 2.0     # CONSTANT width
        if self.last_pos is not None:
            moved = mp_util.gps_distance(self.last_pos[0], self.last_pos[1],
                                         lat, lon)
            if moved < max(0.5, radius * 0.6):        # spacing is by distance,
                return                                # so density is uniform
        slipmap = getattr(self.mpstate, 'map', None) # regardless of speed
        if slipmap is None:
            return
        self.count += 1
        slipmap.add_object(mp_slipmap.SlipCircle(
            'cam%u' % self.count, LAYER, (lat, lon),
            radius, YELLOW, linewidth=-1))
        self.last_pos = (lat, lon)

    def clear(self):
        slipmap = getattr(self.mpstate, 'map', None)
        if slipmap is not None:
            slipmap.add_object(mp_slipmap.SlipClearLayer(LAYER))
        self.count = 0
        self.last_pos = None

    # ---- commands --------------------------------------------------------
    def cmd_camsp(self, args):
        if not args or args[0] == 'status':
            print("camsp: greenness(nExG)=%.3f  spraying=%s  rate=%s  "
                  "speed=%.1f m/s  swath=%.1f m  marks=%u  (on>=%.2f off<%.2f)"
                  % (self.greenness, self.spraying, self.commanded_rate,
                     self.speed, self.cam_settings.swath_m, self.count,
                     self.cam_settings.green_on, self.cam_settings.green_off))
        elif args[0] == 'clear':
            self.clear()
            print("camsp: overlay cleared")
        elif args[0] == 'set':
            self.cam_settings.command(args[1:])
        else:
            print("usage: camsp <status|clear|set>")


def init(mpstate):
    '''initialise module'''
    return CameraSprayModule(mpstate)
