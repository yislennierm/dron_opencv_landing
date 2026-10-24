"""Bounded reference bank with explicit validity and no stale target output."""
from dataclasses import asdict, dataclass, field
import time

import cv2
import numpy as np

from .features import Extractor, Features, mutual_ratio_matches
from .geometry import local_scale, project_point


@dataclass(frozen=True)
class TrackerConfig:
    max_features: int = 600
    max_width: int = 640
    max_height: int = 480
    max_references: int = 12
    min_inliers: int = 16
    min_inlier_ratio: float = 0.5
    min_coverage: float = 0.03
    ratio: float = 0.75
    reprojection_px: float = 3.0
    max_median_error_px: float = 1.5
    max_anisotropy: float = 4.0
    confirmation_frames: int = 3
    max_frame_age_s: float = 0.25
    max_telemetry_age_s: float = 0.05
    max_gap_s: float = 0.5
    max_home_speed_px_s: float = 600.0
    max_reference_disagreement_px: float = 15.0
    keyframe_scale: float = 0.8
    keyframe_interval_s: float = 0.5

    def __post_init__(self):
        if min(self.max_features, self.max_width, self.max_height, self.max_references, self.confirmation_frames) < 1:
            raise ValueError("Feature, image, reference and confirmation limits must be positive")
        if self.min_inliers < 4 or not 0 < self.ratio < 1 or not 0 < self.keyframe_scale < 1:
            raise ValueError("Invalid matching or keyframe thresholds")
        if not 0 < self.min_inlier_ratio <= 1 or not 0 < self.min_coverage <= 1:
            raise ValueError("Invalid confidence thresholds")
        if min(self.reprojection_px, self.max_median_error_px, self.max_anisotropy,
               self.max_frame_age_s, self.max_telemetry_age_s, self.max_gap_s,
               self.max_home_speed_px_s, self.max_reference_disagreement_px, self.keyframe_interval_s) <= 0:
            raise ValueError("Timing and geometry limits must be positive")


@dataclass
class Reference:
    features: Features
    home: np.ndarray
    timestamp_s: float


@dataclass
class Observation:
    timestamp_s: float
    valid: bool = False
    reason: str = "no_reference"
    reference_index: int | None = None
    home_px: list | None = None
    offset_px: list | None = None
    ground_offset_m: list | None = None
    metric_valid: bool = False
    metric_reason: str = "tracking_invalid"
    quality: dict = field(default_factory=dict)
    latency_ms: float = 0.0
    frame_age_s: float = 0.0

    def to_dict(self):
        return asdict(self)


class LandingTracker:
    def __init__(self, config=None, camera=None):
        self.config = config or TrackerConfig()
        c = self.config
        self.extractor = Extractor(c.max_features, c.max_width, c.max_height, camera)
        self.references: list[Reference] = []
        self._previous_home = None
        self._previous_timestamp = None
        self._active_reference = None
        self._confirmations = 0

    def reset(self):
        self.references.clear()
        self._previous_home = self._previous_timestamp = self._active_reference = None
        self._confirmations = 0

    @property
    def reference_bytes(self):
        return sum(r.features.points.nbytes + r.features.descriptors.nbytes + r.home.nbytes
                   for r in self.references)

    def _align(self, reference, live):
        c = self.config
        if reference.features.shape != live.shape:
            return None
        matches = mutual_ratio_matches(reference.features, live, c.ratio)
        if len(matches) < c.min_inliers:
            return None
        source = np.float32([reference.features.points[m.queryIdx] for m in matches])
        target = np.float32([live.points[m.trainIdx] for m in matches])
        H, mask = cv2.findHomography(source, target, cv2.RANSAC, c.reprojection_px,
                                     maxIters=2000, confidence=0.995)
        if H is None or mask is None or not np.isfinite(H).all():
            return None
        keep = mask.ravel().astype(bool)
        count, ratio = int(keep.sum()), float(keep.mean())
        if count < c.min_inliers or ratio < c.min_inlier_ratio:
            return None
        source, target = source[keep], target[keep]
        h, w = live.shape
        norm = np.diag([1/w, 1/h, 1])
        if np.linalg.cond(norm @ H @ np.linalg.inv(norm)) > 1e5:
            return None
        try:
            home = project_point(H, reference.home)
            scale, anisotropy = local_scale(H, reference.home)
            inverse = np.linalg.inv(H)
        except (ValueError, np.linalg.LinAlgError):
            return None
        if anisotropy > c.max_anisotropy or not (0 <= home[0] < w and 0 <= home[1] < h):
            return None
        hull_source, hull_target = cv2.convexHull(source), cv2.convexHull(target)
        coverage = min(cv2.contourArea(hull_source), cv2.contourArea(hull_target))/(w*h)
        if coverage < c.min_coverage:
            return None
        # Avoid extrapolating home beyond the supporting ground correspondences.
        if cv2.pointPolygonTest(hull_source, tuple(map(float, reference.home)), False) < 0:
            return None
        if cv2.pointPolygonTest(hull_target, tuple(map(float, home)), False) < 0:
            return None
        forward = cv2.perspectiveTransform(source[:, None, :], H)[:, 0, :]
        backward = cv2.perspectiveTransform(target[:, None, :], inverse)[:, 0, :]
        errors = np.maximum(np.linalg.norm(forward-target, axis=1), np.linalg.norm(backward-source, axis=1))
        median = float(np.median(errors))
        if median > c.max_median_error_px or not np.isfinite(errors).all():
            return None
        quality = dict(matches=len(matches), inliers=count, inlier_ratio=ratio,
                       coverage=coverage, median_error_px=median,
                       scale_reference_to_live=scale, anisotropy=anisotropy)
        return home, quality

    def _best(self, features):
        candidates = []
        for i, reference in enumerate(self.references):
            alignment = self._align(reference, features)
            if alignment is not None:
                home, quality = alignment
                score = quality['inliers'] * quality['coverage'] / (1 + quality['median_error_px'])
                candidates.append((score, i, home, quality))
        if not candidates:
            return None
        best = max(candidates, key=lambda item: item[0])
        # Competitive references must identify the same physical home point.
        if any(item[0] >= best[0]*0.85 and np.linalg.norm(item[2]-best[2]) > self.config.max_reference_disagreement_px
               for item in candidates):
            return None
        # Keep a good current reference rather than flicker between nearby levels.
        active = next((item for item in candidates if item[1] == self._active_reference), None)
        return active if active is not None and active[0] >= best[0]*0.85 else best

    def map_frame(self, image, timestamp_s, home=None, force=False, *,
                  camera_to_world=None, telemetry_timestamp_s=None):
        """Capture a first/manual keyframe, or auto-capture on validated scale change.

        Initial home is in PROCESSED, RECTIFIED pixel coordinates. Later references
        always inherit it through reference -> live alignment; failure never resets
        home to the image center. Reference storage stops at the configured cap.
        """
        if not np.isfinite(timestamp_s):
            raise ValueError("Frame timestamp must be finite")
        gray, camera = self.extractor.prepare(image)
        features = self.extractor.extract(gray)
        if len(features.points) < self.config.min_inliers:
            return "insufficient_features"
        if self.references and timestamp_s <= self.references[-1].timestamp_s:
            return "out_of_order_frame"
        if len(self.references) >= self.config.max_references:
            return "reference_limit"
        if not self.references:
            h, w = gray.shape
            if home is None and camera_to_world is not None:
                if camera is None:
                    raise ValueError("Setting nadir home from attitude requires calibration")
                if telemetry_timestamp_s is None or not np.isfinite(telemetry_timestamp_s) or abs(timestamp_s-telemetry_timestamp_s) > self.config.max_telemetry_age_s:
                    raise ValueError("Setting nadir home requires synchronized attitude")
                home = camera.nadir_pixel(camera_to_world)
            anchor = np.asarray(home if home is not None else [w/2, h/2], dtype=float)
            if anchor.shape != (2,) or not np.isfinite(anchor).all() or not (0 <= anchor[0] < w and 0 <= anchor[1] < h):
                raise ValueError("Home must be inside the processed image")
            hull = cv2.convexHull(features.points)
            if cv2.pointPolygonTest(hull, tuple(map(float, anchor)), False) < 0:
                return "home_outside_feature_support"
        else:
            # Sequential overlap is required to propagate one physical home point.
            alignment = self._align(self.references[-1], features)
            if alignment is None:
                return "mapping_lost"
            anchor, quality = alignment
            scale = quality['scale_reference_to_live']
            if not force:
                if timestamp_s-self.references[-1].timestamp_s < self.config.keyframe_interval_s:
                    return "mapping"
                if self.config.keyframe_scale < scale < 1/self.config.keyframe_scale:
                    return "mapping"
        self.references.append(Reference(features, anchor.copy(), float(timestamp_s)))
        return "captured"

    def observe(self, image, timestamp_s, *, height_m=None, camera_to_world=None,
                telemetry_timestamp_s=None, now_s=None):
        started = time.perf_counter()
        if not np.isfinite(timestamp_s) or (now_s is not None and not np.isfinite(now_s)):
            raise ValueError("Timestamps must be finite")
        result = Observation(float(timestamp_s))
        def finish(reason):
            result.reason = reason
            result.latency_ms = (time.perf_counter()-started)*1000
            result.frame_age_s = max(0., (now_s if now_s is not None else timestamp_s)-timestamp_s) + result.latency_ms/1000
            if result.frame_age_s > self.config.max_frame_age_s:
                result.valid, result.metric_valid = False, False
                result.home_px = result.offset_px = result.ground_offset_m = None
                result.reason, result.metric_reason = "stale_frame", "tracking_invalid"
            if not result.valid and result.reason != "acquiring":
                self._confirmations = 0
                self._previous_home = None
            return result
        if now_s is not None and now_s < timestamp_s:
            return finish("future_frame")
        if self._previous_timestamp is not None and timestamp_s <= self._previous_timestamp:
            return finish("out_of_order_frame")
        # Reject malformed input before changing temporal tracking state.
        gray, camera = self.extractor.prepare(image)
        if self._previous_timestamp is not None and timestamp_s-self._previous_timestamp > self.config.max_gap_s:
            self._confirmations, self._previous_home = 0, None
        self._previous_timestamp, previous_timestamp = float(timestamp_s), self._previous_timestamp
        if not self.references:
            return finish("no_reference")
        best = self._best(self.extractor.extract(gray))
        if best is None:
            return finish("no_good_alignment")
        _, index, home, quality = best
        result.reference_index, result.quality = index, quality
        if self._previous_home is not None:
            limit = 12 + self.config.max_home_speed_px_s*(timestamp_s-previous_timestamp)
            if np.linalg.norm(home-self._previous_home) > limit:
                return finish("implausible_jump")
        self._active_reference, self._previous_home = index, home
        self._confirmations += 1
        if self._confirmations < self.config.confirmation_frames:
            return finish("acquiring")
        h, w = gray.shape
        result.valid = True
        result.home_px, result.offset_px = home.tolist(), (home-np.array([w/2,h/2])).tolist()
        if camera is None:
            result.metric_reason = "missing_calibration"
        elif height_m is None or camera_to_world is None:
            result.metric_reason = "missing_height_or_attitude"
        elif telemetry_timestamp_s is None or not np.isfinite(telemetry_timestamp_s) or abs(timestamp_s-telemetry_timestamp_s) > self.config.max_telemetry_age_s:
            result.metric_reason = "unsynchronized_telemetry"
        else:
            try:
                result.ground_offset_m = camera.ground_offset(home, height_m, camera_to_world).tolist()
                result.metric_valid, result.metric_reason = True, "ok"
            except ValueError as error:
                result.metric_reason = str(error)
        return finish("ok")
