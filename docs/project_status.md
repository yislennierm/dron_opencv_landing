# Project status and decisions

Updated 2026-10-24. This is the starting point for continuing the work. Detailed
algorithm contracts are in [the baseline](landing_baseline.md), controller
findings in [Betaflight integration](betaflight_integration.md), and proposed
fusion/control policy in [vision architecture](vision_architecture.md).

## Intended feature

Remember natural ground around takeoff, use GPS return to reach home vicinity,
then use downward vision for better final alignment and landing. No printed
marker is required. The eventual target is native vision inside an ESP32-P4
Betaflight build, with board-specific MIPI camera configuration and a proposed
`USE_VISION` build option. Compilation, runtime enablement and flight authority
are separate decisions. No such firmware feature has been implemented yet.

Downward optical flow/local position hold is a second proposed capability.
Recognizing remembered home and measuring frame-to-frame motion are different
outputs; the latter still needs an estimator and validation.

## Implemented and usable on the computer

- Maintained Python `landing` package: oriented multiscale ORB, mutual descriptor
  matching, RANSAC geometry/support checks, bounded reference bank, temporal
  confirmation, calibrated metric projection, webcam, replay and benchmarks.
- Interactive browser simulation: procedural 256 m ground (configurable up to
  4096 m), 3D quad, rendered downward camera, manual cross controls, independent
  altitude controls, brake, return during flight, automatic demo, wind/GPS bias
  and camera-loss controls. Sidebar uses tabs to fit a single viewport.
- Displays actual simulator velocity requests and their source, alongside the
  tracker's image/metric observations. The tracker produces observations;
  simulator guidance turns those observations into velocity requests.
- Eight selectable vision stages with enlargement and pause: input, rectified
  grayscale, ORB, saved reference, matches, inliers, alignment/support, confirmed
  home. These use captured processing data, not a second tracking pass.
- Repeatable trials distinguish successful landing, inaccurate GPS landing and
  failure to acquire home. Unit/integration checks cover tracking, geometry,
  simulator behavior and diagnostic views.

Run `python3 -m landing simulate`, or
`python3 -m landing simulate --no-browser` and open `http://127.0.0.1:8765/`.
Run tests with `python3 -m unittest discover -s tests -q` and trials with
`python3 -m landing sim-trials --seeds 7,42 --output results/simulation_trials.json`.
Generated reports are ignored by Git; retain scenario settings when comparing.

Verification on 2026-10-24 after the attitude-sign correction: **53 tests pass**.
The 10 trials across seeds 7/42 produce eight landings, six within the 15 cm
tolerance, two GPS-only landings outside tolerance and two entirely obscured
return timeouts. No false or inaccurate accepted locks were reported by those
scenarios. These outcomes are limited to this model and scenario set.

The simulator executes real image tracking with simplified kinematic motion and
ideal synchronized attitude/height. It does not execute Betaflight, simulate
motor/ESC dynamics, or demonstrate real landing accuracy. Flat texture excludes
terrain relief and real camera timing/vibration/blur effects. Position-hold
fusion, firmware handoff and embedded performance remain unvalidated.

## Movement and attitude convention

Forward flight lowers the nose; sideways travel banks toward travel. Braking can
require the opposite lean in a physical aircraft. Our simplified simulator
prescribes tilt from velocity, so it does not reproduce acceleration/braking
dynamics. Its previous forward/backward pitch and lateral bank signs were
reversed; both were corrected on 2026-10-24, including camera pose and telemetry.

Simulator body axes are x forward, y left, z up; rotation is
`Rz(yaw) Ry(pitch) Rx(roll)`. Positive simulator pitch lowers the nose. At zero yaw
the body's forward axis is world +x, whereas the downward image's up direction
is world +y. The cross buttons intentionally use image directions. These frames
must be explicitly converted before connecting to Betaflight; simulator angle
signs are not an RC/MSP convention.

## Betaflight integration decisions

USB MSP is available for read-only bench work. A separate onboard processor can
use a dedicated UART with MSP: initial 115200 baud, 8N1, normal noninverted
full-duplex TTL, no hardware flow control, crossed TX/RX and common ground;
verify voltage and board resources. Camera images stay on the vision processor.
Integrated vision can publish measurements internally without a UART.

Stock RC override supplies receiver channels, not home offsets or velocity.
Existing flow/range/GPS inputs are sensor measurements, not a remembered-home
landing interface. The inspected stock source has no dedicated home-vision input.
The preferred extension publishes timestamped, framed, validated observations to
an FC-owned guidance source selector, with explicit transfer/fallback behavior.
Rescue and vision must share one coherent position/attitude/rate/mixer chain;
competing GPS anchors and visual commands must not keep pulling independently.

Sensor fusion is separate from guidance ownership. Existing Betaflight code
gates and weights measurements by health/uncertainty; it has no universal fixed
GPS-versus-vision percentage. GPS course is travel direction, not always nose
heading. Keep existing heading policy initially; visual yaw requires separate
observability/correlation work. There is no justified fixed handoff altitude yet.

## Local source and connected board

| Item | Observed identity |
| --- | --- |
| Stock source | `/home/nene/Documents/betaflight`, commit `1c6716a886be8e9625faad0b27aca6b7af972c4e` |
| Older P4 integration | `/home/nene/Documents/betaesp32`, branch `integration/esp32p4-m1`, inspected commit `74df874f984585b873066191a14283857940b50b`; target `P4BN` / `ESP32P4_BENCH` |
| Connected FireBeetle, read-only MSP | `FP4B` / `FIREBEETLE2P4_BENCH`, API 1.49, `2026.12.0-alpha`, revision `e5071ce`, build Sep 27 2026 19:08:27 |

The older integration is **not the identified source for the connected image**.
The exact FireBeetle source was not found in the inspected local locations.
The user may check other Windows/macOS partitions later; that search is deferred.
Recorded USB identity is Espressif serial `E8:F6:0A:E0:F5:EF`, observed at
`/dev/ttyACM0`; enumerate again rather than assuming the same device path.
MSP status reported gyro/accelerometer flags and a 250 microsecond cycle, with
short status-response round trips near 2 ms. These do not establish flight
qualification, sensor correctness, or available vision processing headroom.
No arming, motor testing, RC override, flashing or configuration writes were done.
Hardware work remains read-only under the existing scope.

## Camera purchased and next milestones

The user purchased the linked InnoMaker OV9281 global-shutter camera; see
[camera selection and bring-up](camera_plan.md) for lens/driver caveats and links.
The exact delivered lens and module revision remain to be confirmed.

1. Compare lens coverage and resolution in repeatable simulation scenarios;
   the current simulator camera is still the generic 640 × 480 calibration.
2. Locate the source/configuration that produced the connected FireBeetle image.
3. Confirm module pinout/power/clock, capture raw frames and calibrate the actual
   lens at the chosen mode. Capture is a milestone before onboard recognition.
4. Establish baseline control-loop timing and memory, then measure camera-only
   effects. Port the selected vision core to C/C++; keep Python as the reference.
5. Qualify bounded native processing, observation-only concurrency, frame age,
   memory, receiver service and control deadlines before enabling guidance.
6. Test estimator/source handoffs, delay/loss, bias and pilot/failsafe behavior
   with actual Betaflight control code in software/HIL before flight guidance.

Do not run the desktop Python/OpenCV application unchanged inside the FC. Initial
native experiments may reduce image size, features and update rate, but require
accuracy checks. High camera FPS and low average CPU load alone do not establish
that control deadlines are met.
