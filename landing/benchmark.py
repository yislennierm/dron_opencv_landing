"""Controlled planar scenes; intentionally separate from real flight validation."""
import cv2
import numpy as np

from .features import Extractor
from .geometry import CameraModel, NADIR_CAMERA_TO_WORLD, plane_homography, project_point
from .replay import summarize
from .tracker import LandingTracker, TrackerConfig


def check_benchmark(report):
    """Regression bounds for these controlled scenes, not a flight acceptance test."""
    failures = []
    for scenario in report['scenarios']:
        if scenario['false_locks'] or scenario['inaccurate_locks']:
            failures.append(scenario['scenario']+': false or inaccurate lock')
        if scenario['visible_frames'] and not scenario['valid_frames']:
            failures.append(scenario['scenario']+': failed to acquire')
        if scenario['ground_error_m'] and scenario['ground_error_m']['maximum'] > 0.02:
            failures.append(scenario['scenario']+': synthetic ground error exceeds 2 cm')
    if failures:
        raise ValueError('; '.join(failures))


def texture(seed=7, width=640, height=480):
    rng = np.random.default_rng(seed)
    image = np.full((height,width), 120, np.uint8)
    for _ in range(1400):
        x,y = rng.integers([0,0], [width,height])
        radius = int(rng.integers(1,7))
        cv2.circle(image, (int(x),int(y)), radius, int(rng.integers(10,245)), -1)
    return image


def rotation(x=0, y=0, z=0):
    return cv2.Rodrigues(np.deg2rad(np.array([x,y,z], float)))[0]


def run_benchmark(image=None):
    if image is None:
        source = texture()
    else:
        source = cv2.imread(str(image), cv2.IMREAD_GRAYSCALE)
        if source is None:
            raise ValueError(f"Cannot read {image}")
        source, _ = Extractor().prepare(source)
    h,w = source.shape
    camera = CameraModel(w,h,500.,500.,w/2,h/2)
    home = np.array([w/2,h/2])
    cases = [
        ('translation', [0.12,-0.08,2], NADIR_CAMERA_TO_WORLD, None),
        ('yaw_45', [0,0,2], rotation(z=45)@NADIR_CAMERA_TO_WORLD, None),
        ('ascent', [0,0,2.8], NADIR_CAMERA_TO_WORLD, None),
        ('tilt_10', [0,0,2], rotation(x=10)@NADIR_CAMERA_TO_WORLD, None),
        ('combined', [0.1,0.04,2.4], rotation(x=5,z=15)@NADIR_CAMERA_TO_WORLD, None),
        ('darker', [0,0,2], NADIR_CAMERA_TO_WORLD, 'dark'),
        ('blur', [0,0,2], NADIR_CAMERA_TO_WORLD, 'blur'),
        ('blank', [0,0,2], NADIR_CAMERA_TO_WORLD, 'blank'),
        ('unrelated', [0,0,2], NADIR_CAMERA_TO_WORLD, 'unrelated'),
    ]
    reports = []
    for name,position,R,effect in cases:
        tracker = LandingTracker(TrackerConfig(max_frame_age_s=2), camera)
        status = tracker.map_frame(source, 0)
        H = plane_homography(camera, [0,0,2], NADIR_CAMERA_TO_WORLD, position, R)
        target = cv2.warpPerspective(source, H, (w,h))
        visible = effect not in ('blank','unrelated')
        if effect == 'dark':
            target = cv2.convertScaleAbs(target, alpha=0.65, beta=10)
        elif effect == 'blur':
            target = cv2.GaussianBlur(target, (7,7), 1.8)
        elif effect == 'blank':
            target = np.full_like(source, 120)
        elif effect == 'unrelated':
            target = texture(123,w,h)
        records = []
        for i in range(1,6):
            timestamp = i/20
            record = tracker.observe(target, timestamp, height_m=position[2],
                camera_to_world=R, telemetry_timestamp_s=timestamp).to_dict()
            record['expected_visible'] = visible
            if visible:
                record['ground_truth_home_px'] = project_point(H,home).tolist()
                record['ground_truth_ground_offset_m'] = (-np.asarray(position[:2])).tolist()
            records.append(record)
        reports.append(dict(scenario=name, mapping=status, **summarize(records)))
    # Real time continuity test: lock, lose texture, then reacquire the same home.
    tracker = LandingTracker(TrackerConfig(max_frame_age_s=2), camera)
    tracker.map_frame(source, 0)
    records = []
    for i in range(1,14):
        visible = not 5 <= i <= 7
        frame = source if visible else np.full_like(source,120)
        result = tracker.observe(frame, i/20).to_dict()
        result['expected_visible'] = visible
        if visible:
            result['ground_truth_home_px'] = home.tolist()
        records.append(result)
    reports.append(dict(scenario='loss_recovery', **summarize(records)))
    return dict(kind='synthetic_planar_benchmark', image_size=[w,h], scenarios=reports,
                limitation='Warped single images do not model parallax, rolling shutter, vibration or real sensor synchronization.')
