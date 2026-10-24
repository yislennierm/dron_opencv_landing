"""Closed-loop, planar camera simulation using the production landing tracker.

Truth renders the camera and scores the flight. Only tracker observations enter
visual guidance. Motion is a kinematic test model, not aircraft dynamics.
"""
from dataclasses import asdict, dataclass
import base64
import math
from pathlib import Path

import cv2
import numpy as np

from .geometry import CameraModel, NADIR_CAMERA_TO_WORLD, camera_to_ground_homography
from .tracker import LandingTracker
from .terrain import GROUND_WIDTH_M, ProceduralGround


@dataclass(frozen=True)
class FlightPlan:
    altitude: float = 4.
    distance: float = math.hypot(7, 4)
    return_yaw: float = 0.
    tilt: float = 0.

    def __post_init__(self):
        for name, low, high in [('altitude', 2, 12), ('distance', 2, 100),
                                ('return_yaw', -180, 180), ('tilt', 0, 15)]:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{name} must be between {low} and {high}')


def attitude_matrix(roll, pitch, yaw):
    """Rz(yaw) Ry(pitch) Rx(roll); body x forward, y left, z up.

    World z is up, so positive pitch lowers the nose. These are simulator
    coordinates, not Betaflight attitude/RC conventions.
    """
    sx, cx = math.sin(roll), math.cos(roll)
    sy, cy = math.sin(pitch), math.cos(pitch)
    sz, cz = math.sin(yaw), math.cos(yaw)
    return np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array(
        [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]) @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])


class LandingSimulation:
    def __init__(self, image=None, seed=7, ground_size=GROUND_WIDTH_M):
        if image is None:
            self.ground = ProceduralGround(seed, ground_size)
            self.ground_width_m = self.ground.width_m
        else:
            self.ground = cv2.imread(str(Path(image)))
            if self.ground is None:
                raise ValueError(f'Cannot read ground photograph: {image}')
            self.ground_width_m = 10.
        self.camera = CameraModel(640, 480, 500., 500., 320., 240.)
        self.pixels_per_m = self.ground.shape[1] / self.ground_width_m if image else None
        self.plan = FlightPlan()
        self.reset()

    def reset(self):
        self.tracker = LandingTracker(camera=self.camera, debug=True)
        self.position = np.array([0., 0., .5])
        self.velocity = np.zeros(3)
        self.phase = 'READY'
        self.time = 0.
        self.attitude = np.zeros(3)
        self.rotation = NADIR_CAMERA_TO_WORLD.copy()
        self.frame_position = self.position.copy()
        self.camera_footprint = []
        self.metrics = dict(observed_frames=0, valid_frames=0, negative_frames=0, false_locks=0,
                            inaccurate_locks=0, max_home_error_m=None, first_lock_s=None)
        self.observation = None
        self.mapping = 'not_started'
        self.error = None
        self.manual_flight = False
        self.return_hold = self.position[:2].copy()
        self.return_height = self.plan.altitude
        self.command = self._command('IDLE', np.zeros(3))
        self.frame = self.render()
        return self.snapshot()

    def start(self, flight=None, manual=False):
        plan = FlightPlan(**(flight or {}))
        if not isinstance(manual, bool):
            raise ValueError('Manual mode must be true or false')
        self.reset()
        self.plan = plan
        self.manual_flight = manual
        self.phase = 'TAKEOFF'
        self.mapping = self.tracker.map_frame(self.frame, self.time, camera_to_world=self.rotation,
                                               telemetry_timestamp_s=self.time)
        return self.snapshot()

    def return_home(self):
        """Interrupt any airborne phase without resetting home or references."""
        if self.phase in ('READY', 'LANDED'):
            raise ValueError('Take off before requesting return')
        if self.phase in ('RETURN CLIMB', 'GPS RETURN', 'ACQUIRING', 'VISUAL APPROACH'):
            return self.snapshot()
        self.return_hold = self.position[:2].copy()
        self.return_height = max(float(self.position[2]), self.plan.altitude)
        self.phase = 'RETURN CLIMB' if self.position[2] < self.return_height-.1 else 'GPS RETURN'
        self.command = self._command('GPS', np.zeros(3))
        return self.snapshot()

    def take_control(self):
        if self.phase in ('READY', 'LANDED'):
            raise ValueError('Take off before taking manual control')
        self.phase = 'MANUAL'
        self.command = self._command('PILOT', np.zeros(3))
        return self.snapshot()

    def render(self, occluded=False):
        if occluded:
            return np.full((480, 640, 3), 65, np.uint8)
        if isinstance(self.ground, ProceduralGround):
            return self.ground.render(self.position, self.camera, self.rotation)
        h, w = self.ground.shape[:2]
        texture = np.array([[self.pixels_per_m, 0, w/2], [0, -self.pixels_per_m, h/2], [0, 0, 1]])
        matrix = texture @ camera_to_ground_homography(self.camera, self.position, self.rotation)
        return cv2.warpPerspective(self.ground, matrix, (640, 480),
                                   flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                                   borderMode=cv2.BORDER_CONSTANT, borderValue=(65, 65, 65))

    def _update_attitude(self, dt):
        target_yaw = math.radians(self.plan.return_yaw) if self.phase not in ('TAKEOFF', 'OUTBOUND', 'MANUAL') else self.attitude[2]
        yaw_error = (target_yaw-self.attitude[2]+math.pi) % (2*math.pi)-math.pi
        self.attitude[2] += float(np.clip(yaw_error, -math.radians(40)*dt, math.radians(40)*dt))
        yaw = self.attitude[2]
        vx, vy = self.velocity[:2]
        body_vx, body_vy = math.cos(yaw)*vx+math.sin(yaw)*vy, -math.sin(yaw)*vx+math.cos(yaw)*vy
        maximum = math.radians(self.plan.tilt)
        # Tilt the upward thrust axis toward travel: forward lowers the nose,
        # left lowers the left side. This is a prescribed velocity-based pose.
        target = np.clip([-body_vy/1.5, body_vx/1.5], -1, 1)*maximum
        self.attitude[:2] += (target-self.attitude[:2])*min(3*dt, 1)
        self.rotation = attitude_matrix(*self.attitude) @ NADIR_CAMERA_TO_WORLD

    def _score_observation(self, occluded):
        observation = self.observation
        self.metrics['observed_frames'] += 1
        point = self.rotation.T @ -self.frame_position
        home = self.camera.matrix @ point
        home = home[:2]/home[2] if home[2] > 0 else [-1, -1]
        visible = not occluded and 0 <= home[0] < 640 and 0 <= home[1] < 480
        self.metrics['negative_frames'] += int(not visible)
        if observation.valid:
            self.metrics['valid_frames'] += 1
            self.metrics['false_locks'] += int(not visible)
            if self.metrics['first_lock_s'] is None and self.phase in ('GPS RETURN', 'ACQUIRING', 'VISUAL APPROACH'):
                self.metrics['first_lock_s'] = self.time
            if observation.metric_valid:
                error = float(np.linalg.norm(np.asarray(observation.ground_offset_m)+self.frame_position[:2]))
                previous = self.metrics['max_home_error_m']
                self.metrics['max_home_error_m'] = error if previous is None else max(previous, error)
                self.metrics['inaccurate_locks'] += int(error > .15)

    def advance(self, *, gps=1.2, wind=.5, vision=True, occluded=False, speed=1., manual_input=None):
        if not all(math.isfinite(float(v)) for v in (gps, wind, speed)):
            raise ValueError('Settings must be finite')
        if not (0 <= gps <= 6 and 0 <= wind <= 3 and .5 <= speed <= 4):
            raise ValueError('Settings outside supported range')
        controls = np.asarray(manual_input if manual_input is not None else [0, 0, 0], dtype=float)
        if controls.shape != (3,) or not np.isfinite(controls).all() or np.any(np.abs(controls) > 1):
            raise ValueError('Manual input must contain three finite values between -1 and 1')
        # Each request advances simulated time, not wall time. Substeps keep
        # camera motion and reference spacing independent of machine throughput.
        steps = math.ceil(speed)
        for _ in range(steps):
            self._step(.1 * speed / steps, gps, wind, vision, occluded, controls)
        return self.snapshot()

    def _command(self, source, velocity):
        """Expose the exact simulator setpoint; the tracker supplies offsets only.

        Camera up/down here describes horizontal motion projected onto image
        axes. Vertical ascent/descent is separate and belongs to the simulator.
        """
        horizontal = self.rotation.T @ np.r_[velocity[:2], 0.]
        return dict(source=source, timestamp_s=self.time,
                    world_velocity_m_s=np.asarray(velocity).tolist(),
                    camera_horizontal_velocity_m_s=horizontal[:2].tolist(),
                    vertical='ASCEND' if velocity[2] > 0 else 'DESCEND' if velocity[2] < 0 else 'HOLD',
                    vertical_source='PILOT' if source == 'PILOT' else 'SIMULATOR',
                    tracker_timestamp_s=self.observation.timestamp_s if source == 'VISION' else None)

    def _step(self, dt, gps, wind, vision, occluded, controls=None):
        if self.phase in ('READY', 'LANDED'):
            return
        controls = np.zeros(3) if controls is None else controls
        if np.any(controls):
            self.phase = 'MANUAL'
        self.time += dt
        self._update_attitude(dt)
        self.frame_position = self.position.copy()
        ground_H = camera_to_ground_homography(self.camera, self.position, self.rotation)
        corners = np.array([[0, 0, 1], [639, 0, 1], [639, 479, 1], [0, 479, 1]]) @ ground_H.T
        self.camera_footprint = (corners[:, :2]/corners[:, 2:]).tolist()
        self.frame = self.render(occluded)
        self.observation = None
        if self.phase == 'TAKEOFF':
            self.mapping = self.tracker.map_frame(self.frame, self.time, camera_to_world=self.rotation,
                                                   telemetry_timestamp_s=self.time)
        else:
            self.observation = self.tracker.observe(
                self.frame, self.time, height_m=float(self.position[2]),
                camera_to_world=self.rotation,
                telemetry_timestamp_s=self.time)
            self._score_observation(occluded)
            if self.phase in ('MANUAL', 'RETURN CLIMB') and self.velocity[2] > 0:
                observed_debug = self.tracker.debug_frame
                self.mapping = self.tracker.map_frame(self.frame, self.time,
                    camera_to_world=self.rotation, telemetry_timestamp_s=self.time)
                self.tracker.debug_frame = observed_debug
        valid = bool(vision and not occluded and self.observation and self.observation.metric_valid)
        target = np.array([0., 0.])
        descending = False
        if self.phase == 'TAKEOFF' and self.position[2] >= self.plan.altitude-.1:
            self.phase = 'MANUAL' if self.manual_flight else 'OUTBOUND'
        if self.phase == 'RETURN CLIMB':
            target = self.return_hold.copy()
            if self.position[2] >= self.return_height-.1:
                self.phase = 'GPS RETURN'
        if self.phase == 'OUTBOUND':
            target = np.array([7., 4.])*self.plan.distance/math.hypot(7, 4)
            if np.linalg.norm(self.position[:2] - target) < .3:
                self.phase = 'GPS RETURN'
        if self.phase in ('GPS RETURN', 'ACQUIRING', 'VISUAL APPROACH'):
            target = np.array([gps, gps*.3])
            if valid:
                self.phase = 'VISUAL APPROACH'
                # No true home coordinate is supplied to visual control.
                target = self.position[:2] + self.observation.ground_offset_m
                descending = np.linalg.norm(self.observation.ground_offset_m) < .15
            elif vision:
                self.phase = 'ACQUIRING' if np.linalg.norm(self.position[:2]-target) < .4 else 'GPS RETURN'
            else:
                self.phase = 'GPS RETURN'
                descending = np.linalg.norm(self.position[:2]-target) < .3
        desired_velocity = np.zeros(3)
        manual_horizontal = np.zeros(2)
        if self.phase == 'MANUAL':
            # Solve world horizontal movement that projects onto the requested
            # camera axes, independent of the orbiting spectator view.
            manual_horizontal = np.linalg.solve(self.rotation[:2, :2].T, controls[:2])
            manual_horizontal *= 1.5/max(1., float(np.linalg.norm(manual_horizontal)))
        for i in range(2):
            desired = float(manual_horizontal[i]) if self.phase == 'MANUAL' else float(np.clip((target[i]-self.position[i])*1.35, -1.5, 1.5))
            desired_velocity[i] = desired
            disturbance = wind*.25*math.sin(self.time*.8+i*2)
            self.velocity[i] += (desired-self.velocity[i])*min(dt*3, 1)+disturbance*dt
        self.velocity[2] = min(.6, (self.plan.altitude-self.position[2])*1.5) if self.phase == 'TAKEOFF' else (
            -min(.35, max(.08, self.position[2]*.2)) if descending else 0.)
        if self.phase == 'RETURN CLIMB':
            self.velocity[2] = min(.6, (self.return_height-self.position[2])*1.5)
        if self.phase == 'MANUAL':
            self.velocity[2] = float(controls[2])*.6
            if self.position[2] >= 12 and self.velocity[2] > 0:
                self.velocity[2] = 0.
        desired_velocity[2] = self.velocity[2]
        source = ('PILOT' if self.phase == 'MANUAL' else 'VISION' if self.phase == 'VISUAL APPROACH' and valid else
                  'GPS' if self.phase in ('RETURN CLIMB', 'GPS RETURN', 'ACQUIRING') else 'FLIGHT PLAN')
        self.command = self._command(source, desired_velocity)
        self.position += self.velocity*dt
        self.position[2] = min(12., self.position[2])
        if self.position[2] <= .5 and (descending or self.velocity[2] < 0):
            self.position[2] = .5
            self.phase = 'LANDED'
            self.error = float(np.linalg.norm(self.position[:2]))
            self.velocity[:] = 0
            self.command = self._command('IDLE', np.zeros(3))

    def snapshot(self):
        observation = self.observation.to_dict() if self.observation else None
        return dict(p=self.position.tolist(), v=self.velocity.tolist(), phase=self.phase,
                    time=self.time, observation=observation, mapping=self.mapping,
                    references=len(self.tracker.references), error=self.error,
                    distance=float(np.linalg.norm(self.position[:2])), ground_width_m=self.ground_width_m,
                    flight=asdict(self.plan), attitude_deg=np.rad2deg(self.attitude).tolist(),
                    body_to_world=attitude_matrix(*self.attitude).tolist(),
                    camera_footprint=self.camera_footprint, metrics=self.metrics.copy(),
                    command=self.command.copy())

    def camera_jpeg(self, stage='home'):
        from .vision_debug import stage_frame
        display, info = stage_frame(self, stage)
        ok, data = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if not ok:
            raise ValueError('Cannot encode camera image')
        return base64.b64encode(data).decode('ascii')
