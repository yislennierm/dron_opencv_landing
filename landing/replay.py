"""Timestamped frame/video replay with optional ground-truth evaluation."""
import json
from pathlib import Path

import cv2
import numpy as np

from .geometry import CameraModel
from .tracker import LandingTracker, TrackerConfig


def summarize(records, home_tolerance_px=5.0):
    observations = [r for r in records if 'valid' in r]
    valid = [r for r in observations if r['valid']]
    home_errors, metric_errors, latency = [], [], []
    false_locks, negative_frames, inaccurate_locks = 0, 0, 0
    missed_visible, visible_frames = 0, 0
    recovery_frames, recovery_times = [], []
    pending_recovery = None
    pending_count = 0
    for r in observations:
        latency.append(r['latency_ms'])
        visible = r.get('expected_visible')
        if visible is False:
            negative_frames += 1
            false_locks += int(r['valid'])
            pending_recovery = None
            pending_count = 0
        elif visible is True:
            visible_frames += 1
            missed_visible += int(not r['valid'])
            if pending_recovery is None and not r['valid']:
                pending_recovery, pending_count = r['timestamp_s'], 0
            if pending_recovery is not None:
                pending_count += 1
                if r['valid']:
                    recovery_frames.append(pending_count)
                    recovery_times.append(r['timestamp_s']-pending_recovery)
                    pending_recovery = None
        if r['valid'] and r.get('ground_truth_home_px') is not None:
            error = float(np.linalg.norm(np.asarray(r['home_px'])-r['ground_truth_home_px']))
            home_errors.append(error)
            inaccurate_locks += int(error > home_tolerance_px)
        if r['metric_valid'] and r.get('ground_truth_ground_offset_m') is not None:
            metric_errors.append(float(np.linalg.norm(np.asarray(r['ground_offset_m'])-r['ground_truth_ground_offset_m'])))

    def stats(values):
        if not values:
            return None
        return dict(count=len(values), median=float(np.median(values)),
                    p95=float(np.percentile(values, 95)), maximum=float(np.max(values)))
    return dict(frames=len(observations), valid_frames=len(valid),
                negative_frames=negative_frames, false_locks=false_locks,
                false_lock_rate=false_locks/negative_frames if negative_frames else None,
                visible_frames=visible_frames, missed_visible_frames=missed_visible,
                inaccurate_locks=inaccurate_locks, home_tolerance_px=home_tolerance_px,
                home_error_px=stats(home_errors), ground_error_m=stats(metric_errors),
                latency_ms=stats(latency), recovery_frames=recovery_frames,
                recovery_s=recovery_times)


def run_replay(manifest_path, output=None):
    path = Path(manifest_path)
    manifest = json.loads(path.read_text())
    camera = CameraModel(**manifest['camera']) if manifest.get('camera') else None
    tracker = LandingTracker(TrackerConfig(**manifest.get('config', {})), camera)
    cap = cv2.VideoCapture(str(path.parent/manifest['video'])) if manifest.get('video') else None
    records = []
    last_timestamp = -float('inf')
    try:
        for frame in manifest['frames']:
            timestamp = float(frame['timestamp_s'])
            if not np.isfinite(timestamp) or timestamp <= last_timestamp:
                raise ValueError("Replay timestamps must be finite and strictly increasing")
            last_timestamp = timestamp
            if cap is not None:
                index = frame['frame_index']
                if not isinstance(index, int) or index < 0:
                    raise ValueError("Video frame_index must be a nonnegative integer")
                cap.set(cv2.CAP_PROP_POS_FRAMES, index)
                ok, image = cap.read()
                if not ok:
                    raise ValueError(f"Cannot decode video frame {index}")
            else:
                image = cv2.imread(str(path.parent/frame['image']))
                if image is None:
                    raise ValueError(f"Cannot read image {frame['image']}")
            mode = frame.get('mode', 'track')
            if mode in ('map', 'capture'):
                record = dict(timestamp_s=timestamp, mapping=tracker.map_frame(
                    image, timestamp, home=frame.get('home_px'), force=mode == 'capture',
                    camera_to_world=frame.get('camera_to_world'), telemetry_timestamp_s=frame.get('telemetry_timestamp_s')),
                    references=len(tracker.references))
            elif mode == 'track':
                record = tracker.observe(image, timestamp, height_m=frame.get('height_m'),
                    camera_to_world=frame.get('camera_to_world'),
                    telemetry_timestamp_s=frame.get('telemetry_timestamp_s'),
                    now_s=frame.get('received_timestamp_s', timestamp)).to_dict()
                for key in ('expected_visible', 'ground_truth_home_px', 'ground_truth_ground_offset_m'):
                    if key in frame:
                        record[key] = frame[key]
            else:
                raise ValueError(f"Unknown frame mode: {mode}")
            records.append(record)
    finally:
        if cap is not None:
            cap.release()
    report = dict(summary=summarize(records, manifest.get('home_tolerance_px', 5.0)),
                  reference_count=len(tracker.references), reference_payload_bytes=tracker.reference_bytes,
                  records=records)
    if output:
        Path(output).parent.mkdir(parents=True,exist_ok=True)
        Path(output).write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report
