# Built-in vision: proposed architecture and sensor policy

Design discussion, 2026-10-24. This describes the intended eventual flight
controller feature, not implemented firmware or a qualified camera/board port.
The final arrangement is an onboard processor with a downward-facing MIPI-CSI
camera, preferably global shutter. The external USB/UART MSP path is useful for
bench development; it is not required between modules inside the same firmware.

The user has since purchased an InnoMaker OV9281; the selected hardware,
driver modes and unresolved lens/compatibility details are in
[the camera plan](camera_plan.md). [Project status](project_status.md) records
the connected FireBeetle identity separately from the older local P4 build.

## Build, board, and runtime configuration

Proposed `USE_VISION` compiles the vision subsystem. Board configuration declares
the available camera interface/sensor, control bus and pins, reset/power controls,
clocking, memory/DMA requirements, and camera mounting orientation. Sensor setup
must select a supported resolution/format/frame cadence and exposure timestamp
strategy. Intrinsics/distortion calibration belong to the actual camera/lens and
resolution, and the camera/body lever arm must be represented.

Board capability is separate from runtime authorization. Compiling vision does
not enable guidance. Runtime configuration may independently allow visual motion
measurements, visual position hold, and visual landing. Proposed feature names
and parameters are design placeholders, not stock Betaflight settings.

A global shutter reduces rolling-shutter distortion; it does not eliminate motion
blur, exposure latency, vibration, height/attitude errors or calibration needs.
MIPI specifies a camera transport, not the camera calibration or control policy.
No hardware driver or ESP32-P4 scheduling change is part of this design review.

## Separate two camera products

| Product | Measurement/reference | Intended use | Current repo capability |
| --- | --- | --- | --- |
| Visual motion / odometry | Frame-to-frame translation/rotation, optionally scaled horizontal velocity | Local position hold, low-speed drift reduction, velocity-estimator aid | A separate motion estimator and its validation are still needed. |
| Remembered-home localization | Current view matched against the saved takeoff-area map | Precision final approach and a local home reference | `LandingTracker` implements this planar recognition/offset baseline. |

Home recognition is not required to estimate local optical flow. Conversely,
recognizing home does not by itself establish a continuously valid velocity
measurement over the whole flight. Position hold may use a fresh local anchor
near the current location without seeing the takeoff area.

Monocular flow has scale ambiguity: metric velocity needs reliable height above
the observed plane or a separately validated visual-inertial scale estimator.
Current home-to-meter projection also needs height and synchronized attitude.
A ground image alone does not provide an absolute north heading. Relative visual
yaw may constrain drift, but needs a dedicated observable measurement model and
must not be treated as a compass or inferred directly from a general homography.

## What standard Betaflight already does

Checked local upstream commit `1c6716a886be8e9625faad0b27aca6b7af972c4e`.

- [IMU heading corrections](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/flight/imu.c#L697)
  use gyro propagation plus magnetometer/GPS-course corrections. Validated mag
  use is gated by `compassEnabledAndCalibrated()` including `trust_mag`. The code
  sums correction terms rather than selecting a single fixed-priority sensor.
- [GPS heading gain](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/flight/imu.c#L420)
  depends on ground speed, yaw motion, roll and forward pitch. With a trusted
  calibrated mag, the ordinary GPS-course gain is reduced by a factor of ten.
  Initialization/heading-validity logic is separate. GPS course is travel
  direction, not automatically aircraft heading, particularly in hover,
  sideways/backward movement, or wind.
- [GPS position/velocity fusion](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/flight/position_estimator.c#L760)
  uses measurement variances influenced by reported GPS accuracy.
- [Optical-flow fusion](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/flight/position_estimator.c#L1148)
  checks sensor health, source policy, valid range/height and timing; flow quality
  influences measurement variance. GPS_ONLY/OPTICALFLOW_ONLY/AUTO gate eligible
  sources. These are estimator policies, not two independent motor controllers.

Exact policies differ by release/build. These facts establish useful integration
patterns, not a complete new vision covariance model or robustness guarantee.

## Estimation weighting versus flight authority

**Estimation:** multiple valid sensors may contribute simultaneously. A candidate
measurement is checked for timestamp age, calibration, geometry, observability,
quality and innovation consistency before it is fused. Accepted uncertainty
controls influence; rejected data has no influence. A fixed percentage such as
70% vision / 30% GPS is not an appropriate general policy.

Vision inlier count and reprojection residual alone are not covariance. We need
labeled recordings and noise/outlier characterization across heights, lighting,
blur, texture, tilt and speeds. Measurements derived from the same images and
IMU samples are correlated. Do not fuse flow, home pose and attitude-derived
world offsets as independent high-confidence measurements and double-count them.
The current pipeline uses attitude to project home; feeding that projection back
as independent attitude evidence would create circular information.

**Flight authority:** one coherent guidance/control path selects the target and
feeds the shared attitude/rate/mixer loops. A blended state estimate is compatible
with exclusive guidance ownership. GPS return and visual landing must not each
write competing roll/pitch/throttle commands.

## Proposed operational policy

| Situation | Eligible measurements and guidance |
| --- | --- |
| Manual flight | Existing pilot/stabilization policy; vision may observe/map but gains no control merely by being compiled or producing features. |
| Local position hold | Healthy visual motion can improve horizontal velocity/local hold within its height/texture envelope. GPS may remain fused when permitted and consistent. |
| Return from distance | GPS supplies the global route/home vicinity. Camera motion may aid estimation if valid; no assumed capability to navigate home from arbitrary ground imagery. |
| Home acquisition | Require multiple consistent remembered-home observations, usable height, synchronized attitude, bounded age and agreement/innovation checks. |
| Final alignment/descent | Use the validated visual home reference in the local feedback/guidance path. GPS can remain available globally without pulling the local landing target back to a biased GPS home point. Descent is gated by alignment and measurement health. |
| Visual degradation/loss | Reject stale/invalid visual measurements, increase uncertainty as justified, and execute an explicit guidance fallback/abort policy. Prevent target jumps and stale corrections. |
| Heading support | Keep existing IMU/mag/GPS policy initially. Add relative visual yaw only after its observability, calibration and measurement correlation are validated. |

There is no universal "GPS until 10 m, then vision" threshold. Acquisition needs
home visibility and sufficient ground support; usable height depends on lens,
texture, resolution, speed, exposure and the range source. Configure measured
operating limits plus quality gates and hysteresis. Descent permission is separate
from sensor fusion eligibility. A camera can be useful for velocity while unable
to identify home, or identify home without having a qualified velocity output.

On handoff, represent the home/reference consistently in the local estimator and
controller frame; manage position anchors/integrals and limit acceleration/tilt.
The camera/body offset matters at touchdown. Camera failure should not reset the
global estimator or overwrite GPS home to make fallback appear to work.

## Execution boundary and next development work

Camera acquisition publishes timestamped frames. Bounded vision processing
publishes typed measurements with validity/quality. The estimator consumes fresh
measurements in the agreed frames; the flight-mode/guidance state machine decides
what target is authorized. Existing stabilization and actuator ownership remain.
Image processing should not block the gyro/rate-control task. Core placement,
buffers, worst-case latency and missed-frame behavior require measurement later.

Next steps in this repo:

1. Formalize separate motion, home-localization and eventual relative-yaw outputs,
   with frame/time/calibration contracts and explicit validity.
2. Add measurement delay, height error, heading bias, GPS bias and camera loss to
   repeatable simulation scenarios; retain actual tracking outcomes.
3. Add a local motion/hold experiment and compare it with independent truth.
4. Prototype estimator/authority transitions and record rejection reasons,
   measurement age, uncertainty, target ownership and transfer transients.
5. Only then choose noise models/operating thresholds and map the validated policy
   into the selected standard Betaflight control code and board camera backend.

These are proposed stages, not tasks already completed by the kinematic simulator.

## Embedded port and resource qualification

The Python/NumPy/OpenCV application does not run unchanged inside Betaflight.
Port only the required vision/geometry/state-estimation algorithms to native
C/C++, retaining Python replay/benchmarks as the executable reference. Browser
simulation, terrain generation, diagnostic image encoding, plotting and test
reports stay on the computer. A build flag alone does not create an embedded
runtime or establish a CPU/memory budget.

Desktop defaults need explicit reduction or qualification:

| Resource | Current baseline cost / implication |
| --- | --- |
| Two 640 × 480 grayscale frames | 614400 bytes = 600 KiB, excluding camera raw/ISP buffers, pyramids and scratch. |
| Two 320 × 240 grayscale frames | 153600 bytes = 150 KiB; a useful first experiment, not an accuracy guarantee. |
| Twelve references × 600 features | About 281.4 KiB of point/descriptor/home arrays alone. Embedded layout/overhead may differ. |
| Exhaustive mutual matching | Up to 2 × 600 × 600 × 12 = 8.64 million 256-bit descriptor-pair comparisons per frame. |
| Robust geometry | Up to 2000 configured RANSAC iterations per attempted reference; actual runtime is data-dependent. |
| Feature cap | Grid selection happens after full ORB detection/description, so retaining 600 features does not bound all earlier extraction work to 600 points. |

These are workload/storage calculations from code, not measured P4 timings.
A first experimental native profile could process 320 × 240 luma, 100–200
features, 5–10 frames/s, and a bounded number of candidate references per update.
Reference search may be spread across frames. All such changes must be checked
for acquisition, scale/rotation robustness, competing-reference disagreement,
false locks, accuracy and recovery; fewer features/references are not a free
optimization. Preserve oriented/multiscale descriptor behavior or qualify any
replacement against the same scenes.

Capture should use bounded DMA buffers with explicit ownership. Process the
latest available frame and discard obsolete queued frames instead of allowing
latency to grow. Use suitable camera/ISP output formats/resolutions where
supported, and include all raw/ISP buffers in memory accounting. ISP/PPA/DMA can
offload supported capture, scaling and pixel operations; they do not imply a
hardware implementation of ORB matching or homography estimation. Check the
actual sensor, silicon revision and pinned IDF API:
[IDF 5.4.3 camera driver](https://docs.espressif.com/projects/esp-idf/en/v5.4.3/esp32p4/api-reference/peripherals/camera_driver.html),
[IDF 5.4.3 PPA](https://docs.espressif.com/projects/esp-idf/en/v5.4.3/esp32p4/api-reference/peripherals/ppa.html).

Flight/receiver deadlines determine the vision budget. Example only: 5 ms of
vision CPU time every 100 ms averages 5% of one core. That average alone says
nothing about interrupt blocking, deadline misses, bus/cache contention, or
preempted frame latency. Set budgets only after measuring the baseline flight
and receiver workloads and their required margin.

Native vision needs bounded work units with deadline/cancellation checks between
feature batches, descriptor-match batches and robust-estimation iterations.
Checking elapsed time only after a monolithic long function returns does not
enforce its CPU budget. Bound all loops, queues, allocations and callback work;
preallocate steady-state buffers, keep callbacks/critical sections short, and
let higher-priority control work preempt the vision worker. Publish observations
through a coherent nonblocking mailbox, enforce observation age at consumption,
and invalidate/drop work that misses its budget. Guidance follows its explicit
vision-loss policy when results are unavailable.

The previously inspected local P4 bench build has PSRAM disabled, CPU configured
to 360 MHz, BF on HP0 and an ELRS task on HP1. These are facts about that build,
not proof that the hardware lacks PSRAM or that no alternative scheduling can
work. Do not assume an idle core. PSRAM enablement, DMA/cache correctness and
shared memory bandwidth need qualification before using them. A successfully
booting sensorless bench image does not establish gyro/PID/receiver deadline
margin under a flight workload.

Qualification sequence, with actuators remaining unavailable during bench work:

1. Record baseline control-loop periods, jitter/deadlines, task execution times,
   RX/failsafe service timing, memory/stack headroom and watchdog behavior. A real
   gyro workload is needed for flight-loop timing evidence; sensorless timing is
   a separate bench measurement.
2. Feed saved image fixtures into a standalone P4 native vision benchmark; record
   per-stage CPU time, end-to-end age, heap/stack use and results against Python.
3. Add camera capture without vision computation; measure DMA/ISR/memory effects.
4. Add vision in observation-only mode, then test concurrent RF, USB/logging and
   worst-case scenes such as dense/repetitive texture, blur, low texture and loss.
5. Require an explicit timing/memory report with vision off/on. No added control
   deadline misses within the tested workload envelope; bounded memory and queues;
   no RX degradation; reported dropped/late frames and invalidation behavior.
6. Enable estimator/guidance consumption only after those gates and transfer/loss
   tests pass. CPU utilization and vision FPS alone are not acceptance criteria.

No native P4 vision throughput or flight schedulability has been measured yet.
