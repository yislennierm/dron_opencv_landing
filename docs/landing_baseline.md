# Landing baseline: model and validation

## Scope

Recognize a remembered natural-ground home point with a downward-looking camera
once the takeoff area is visible. The working model is a static, approximately
flat ground plane with sufficient distinct texture and overlap between views.
Large-scale navigation back to that area, obstacle detection, landing-surface
suitability, altitude estimation, and flight control are separate problems.

The Python baseline exists to quantify when this model works and fails before
choosing an embedded implementation. No performance result on the development
computer is an ESP32-P4 performance prediction.

## Coordinate contract

- `H_reference_to_live` maps saved, rectified image coordinates to current,
  rectified image coordinates. Home propagation always applies that direction.
- Image coordinates are in the bounded processing image, with x right and y
  down. By default processing fits inside 640 x 480 without enlarging the input.
- Calibrations describe the actual input resolution. Intrinsics are scaled with
  the processing image; OpenCV pinhole distortion is removed before extraction.
  A resolution mismatch is rejected rather than guessed to be a resize or crop.
- World coordinates use a local Cartesian frame, z up. The zero-yaw nadir camera
  maps image right to +x and image up to +y. `camera_to_world` is a proper 3 x 3
  rotation including aircraft attitude AND camera mounting orientation.
- Metric displacement is **from the current camera to home**, expressed in local
  world x/y meters. It is not a velocity, motor command, body-frame error, or a
  complete aircraft position estimate. A future integration must handle the
  camera/body lever arm and transform into its control frame.
- `height_m` is the CURRENT camera height above the z=0 landing plane. It is
  neither a saved reference height nor an uncorrected slant range. A tilted
  rangefinder needs conversion to vertical ground-plane height.

The calibrated target ray is `camera_to_world @ inv(K) @ [u, v, 1]`. Intersect it
with the ground using current height. Near-horizon, upward-pointing, or invalid
rays are rejected. Camera tilt alone can move home in the image while producing
zero horizontal ground error; the tests cover this case.

Without an explicit anchor, first capture uses synchronized camera attitude and
calibration to choose the ground nadir if those inputs are supplied. Otherwise
it uses the image center, appropriate to an explicitly level-camera bench test.
An explicit `home_px` selects a ground point in the processed, rectified image.
The camera origin and aircraft origin need not be the same physical location.

## Tracking and mapping decisions

The feature extractor runs full ORB orientation and multiscale detection, then
selects features across a grid. It retains at most 600 features per reference.
Descriptors use mutual nearest-neighbor ratio matching with Hamming distance.
RANSAC estimates reference-to-live alignment with a fixed iteration limit.

An accepted alignment needs sufficient inliers and inlier ratio, spatial support
in both views, low symmetric reprojection error, a nondegenerate transform, and
reasonable local anisotropy. Home must be inside both the supporting inlier hull
and the live image. Competitive references that disagree substantially on home
are rejected. These are configurable rejection rules, not calibrated confidence
probabilities or a guarantee against repeated-pattern false locks.

Rectification uses cached remap tables and masks invalid border regions so that
synthetic black image borders cannot serve as remembered ground features.

Three temporally consistent observations establish lock. Implausible jumps,
stale/out-of-order frames, missing alignment, and long frame gaps invalidate or
restart acquisition. Invalid observations carry no home or ground correction.
Consumers must also enforce observation age at the time they use an output.

Mapping propagates one physical home point through consecutive validated views.
It never substitutes the image center after a failed match. Auto-capture uses
the local Jacobian scale in the same reference-to-live direction; it adds a
reference after a 20% scale decrease or corresponding increase, subject to a
minimum time interval. This spacing is a starting hypothesis to validate against
real ascent recordings. Horizontal movement alone does not trigger capture.

The bank stops at 12 references and reports `reference_limit`; it does not silently
discard the original home view. Only point coordinates, binary descriptors, home,
and a timestamp are retained. Maximum array payload is about 288 KB with defaults
(12 x [600 x (8 + 32) + 16] bytes). This excludes Python objects, image working
buffers, pyramids, matching/RANSAC scratch memory, and telemetry.

## Observation contract

`valid` means the image-based home estimate passed the configured gates. It is
independent of `metric_valid`. A valid image estimate can have no meter output
because calibration, current height, attitude, or synchronized telemetry is
missing. `reason` and `metric_reason` explain each decision. Quality contains
inlier count/ratio, coverage, reprojection error, local scale, and anisotropy.

Timestamp values are seconds in one shared time base. Telemetry must be within
50 ms of the exposure timestamp by default. Frame age includes supplied receipt
delay and processing time; the default limit is 250 ms. These are initial test
settings, not derived flight-control limits. Webcam timestamps are HOST RECEIPT
times because OpenCV does not provide reliable sensor exposure timestamps here.
`--assume-level --height ...` provides explicit bench assumptions, not measured
flight telemetry.

## Replay format

`python -m landing replay manifest.json --output report.json` accepts:

```json
{
  "video": "descent.avi",
  "camera": {"width": 640, "height": 480, "fx": 500, "fy": 500, "cx": 320, "cy": 240, "distortion": []},
  "frames": [
    {"frame_index": 0, "timestamp_s": 0.0, "mode": "map", "home_px": [320, 240]},
    {"frame_index": 1, "timestamp_s": 0.05, "height_m": 2.0,
     "camera_to_world": [[1,0,0],[0,-1,0],[0,0,-1]],
     "telemetry_timestamp_s": 0.05,
     "expected_visible": true, "ground_truth_home_px": [325,240]}
  ]
}
```

Calibration numbers and labels above illustrate the schema. For image sequences,
omit `video` and use `image` instead of `frame_index` in each frame. Paths are
relative to the manifest. Timestamps must increase strictly. `mode` defaults to
`track`; `map` permits automatic capture and `capture` forces a validated capture.
All frames, including acquisition frames, must be listed to evaluate recovery.
Optional `received_timestamp_s` models capture/transport delay; optional
`ground_truth_ground_offset_m` labels world x/y displacement.

Reports include every observation and a summary:

- Home error in processed pixels and ground error in meters, when labeled AND
  valid. Rejections are not assigned zero error.
- Negative frames (`expected_visible: false`) and false locks on those frames.
- Visible but invalid frames, including initial confirmation frames; accepted
  predictions more than the configured pixel tolerance away are counted
  separately as `inaccurate_locks`.
- Initial acquisition/recovery frames and elapsed seconds, measured from the
  first invalid visible frame until lock. Unrecovered segments contribute missed
  visible frames but no completed recovery duration.
- Processing latency, excluding file/video decoding. It includes feature
  extraction, reference matching, geometric validation, and metric projection.

The sample replay reuses `sample.jpg` and validates plumbing only. No real
ascent/descent dataset has been recorded or validated in this repository yet.

## Validation sequence

1. Run unit/integration tests for coordinate direction, height scale, yaw, tilt,
   initial home selection, reference propagation, loss/recovery, degenerate
   geometry, stale data, storage limits, and image/video replay.
2. Run the generated-texture benchmark and repeat with `sample.jpg`. The rendered
   plane provides known home location and camera displacement across translation,
   yaw, ascent, tilt, combined motion, dimming, and blur, plus negative scenes.
   `--check` fails if a visible scenario never acquires, any labeled negative
   locks, accepted home error exceeds 5 pixels, or synthetic metric error exceeds
   2 cm. These bounds apply only to the generated cases.
3. Record handheld/rig footage over textured ground with measured camera height
   and separately surveyed home. Include concrete, gravel, grass, repetitive
   paving, partial occlusion, exposure changes, and deliberate tracking loss.
4. Add calibrated, timestamped attitude and independent position measurements.
   Measure error distributions, outliers, false-lock rate, recovery, and frame age
   across the intended height/tilt/speed range. Split tuning and evaluation runs.
5. Set operating-envelope and rejection thresholds from those recordings, then
   profile bounded memory and worst-case execution on the selected ESP32-P4 board.

Synthetic cases do not model parallax from uneven terrain, rolling shutter,
vibration, motion-dependent exposure, moving vegetation, sensor delay, or
accumulated mapping drift. A full flight controller will additionally need an
attitude/height estimator, output uncertainty characterization, and deterministic
control scheduling. None of these is established by a successful homography.

## Geometry references

- [OpenCV homography geometry](https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html)
  explains the planar-scene model and the scale ambiguity of recovered motion.
- [OpenCV camera model](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
  defines intrinsics, distortion, and coordinate transformations.
- [OpenCV ORB implementation, 4.12.0](https://github.com/opencv/opencv/blob/4.12.0/modules/features2d/src/orb.cpp)
  distinguishes full feature detection/orientation from computing descriptors
  at externally supplied keypoints.
