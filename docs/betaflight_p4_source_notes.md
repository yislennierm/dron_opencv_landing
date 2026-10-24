# Betaflight visual landing: source review and proposed integration

Reviewed 2026-10-24. This is a design/source review, not an implemented Betaflight
integration. The simulator's arrows and velocity targets are illustrative guidance.
No firmware, flight-controller configuration, or motor commands were changed.

## Actual local P4 source identified

**Subsequent hardware identification:** the attached FireBeetle reports `FP4B` /
`FIREBEETLE2P4_BENCH`, `2026.12.0-alpha`, API 1.49, revision `e5071ce`, build
Sep 27 2026 19:08:27. This differs from the older `P4BN` image below. Its matching
source has not been located; checking other OS partitions is deferred to the
user. The source candidates and next-work list below describe the older image,
not an established integration path for the attached firmware. See
[current project status](project_status.md) before continuing hardware work.

Follow-up inspection found these separate checkouts:

| Location | Identity and role |
| --- | --- |
| `/home/nene/Documents/betaflight` | Upstream checkout at `1c6716a886be8e9625faad0b27aca6b7af972c4e`; its existing devcontainer edit was left unchanged. |
| `/home/nene/Documents/betaesp32` | Integration project, branch `integration/esp32p4-m1`, inspected at `74df874f984585b873066191a14283857940b50b`. |
| `/home/nene/Documents/betaesp32/upstream/betaflight` | Pinned pristine Betaflight `617e84f11fa165c87ceb91c6d6d0a279cdf2d5ef`. |
| `/home/nene/Documents/betaesp32/build/betaflight-p4` | Detached P4 worktree at the same pin plus exported integration patches; this is the BF source consumed by the IDF bench build. |

The combined build identifies itself as `ESP32P4_BENCH` / `P4BN`. Preprocessing
`gps_rescue_multirotor.c` with its existing `compile_commands.json` command
confirmed `USE_GPS`, `USE_GPS_RESCUE`, `USE_POSITION_HOLD`, and
`USE_ALTITUDE_HOLD`, but **`ENABLE_FLIGHT_PLAN=0` and `ENABLE_RESCUE_PLAN=0`**.
Therefore the examined P4 build uses the **legacy Rescue path**. Compile-time
sensor support is not evidence of working attached sensors.

The current `.bin` matches the project's recorded installed artifact
`build/artifacts/p4-responsive-daeb8ab/p4_bf_bench.bin`: 994208 bytes, SHA256
`1dc93dbecd8a4b8437b66a80b85fd0e7e26b94337377b0eeb6fc47ec67cd37b4`.
The current and archived ELFs also match. Symbol inspection confirms
`gpsRescueUpdate`, `positionControl`, `updatePosHold`, and `updateAltHold`;
no `flightPlanNav` symbols were found. This identifies the archived build,
not a fresh readback of the currently attached hardware.

The local source offers more concrete integration candidates than the earlier
release-level review:

- `build/betaflight-p4/src/main/flight/gps_rescue_multirotor.c:270`:
  `calculateTargetStep()` supplies the legacy GPS horizontal target.
- `build/betaflight-p4/src/main/flight/autopilot_multirotor.c:834`:
  `xySelectMode()` chooses the shared horizontal-control mode.
- The same file at line 981: `positionControl()` runs the shared feedback law;
  the mode feeders control targets, anchor policy, and integral policy.
- `build/betaflight-p4/src/main/flight/pid.c:587`: Rescue's attitude targets
  enter `pidLevel()`; a second controller should not overwrite these separately.

A proposed visual-landing mode/source must coordinate the Rescue target feeder
and shared XY mode/error selection. Changing one writer while another continues
updating the GPS anchor is insufficient. This is a design candidate, not a patch.
The local `altitudeControl()` uses a `timeUs_t` interval, unlike the floating-point
seconds argument in the initially reviewed release; use the actual local API.

This image is explicitly a bench integration: `platform/esp32p4/bench/target.h`
disables DShot/PWM outputs and sets the motor protocol disabled. The entry point
retains arming blocks and a sensorless scheduler budget/yield, not a qualified
flight loop. USB/MSP operation and an internal receiver interface are documented;
these do not establish gyro, GPS landing, RF reception, or actuator capability.

BF's IDF main task is pinned to HP0, while the combined ELRS receiver task is
pinned to HP1. Do not assume the second HP core is free for vision. Keep initial
vision processing on the computer; design a bounded observation receiver and
measure concurrency before choosing on-chip placement. No camera acquisition
path has been established in this review.

## Earlier upstream release review

Before locating the P4 integration, source research used upstream tag `2026.6.2`,
commit [`e0b7bb01b17b21351057e9ead2d1ab39dd44fa16`](https://github.com/betaflight/betaflight/tree/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16),
matching the release string previously read from the conventional F722/H743
controllers. The following pinned links describe that release, not the P4
worktree. Local source/build evidence above takes precedence for P4 work.

## What the inspected firmware actually does

There are two Rescue paths in this release. The build configuration matters:

- [common_post.h, lines 876–896](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/target/common_post.h#L876)
  derives `ENABLE_FLIGHT_PLAN` and `ENABLE_RESCUE_PLAN` from compiled features.
- In the legacy path, `gpsRescueUpdate()` selects a Rescue phase, updates the
  horizontal target through `moveTargetLocation()`, and calls `altitudeControl()`.
  See [gps_rescue_multirotor.c](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/flight/gps_rescue_multirotor.c#L570).
- With `ENABLE_RESCUE_PLAN`, `gpsRescueUpdate()` maintains sensor/availability
  state, while a mission performs the return. The generated plan climbs at the
  current location, flies over GPS home, then lands at home. See
  [the Rescue plan builder](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/flight/flight_plan_nav.c#L917)
  and [mode selection in core.c](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/fc/core.c#L1062).

The shared downstream path is:

| Layer | Inspected implementation | Role |
| --- | --- | --- |
| Mission landing | [`startLanding()` / `updateLanding()`](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/flight/flight_plan_nav.c#L815) | Sets a landing target; manages descent/touchdown policy. |
| Navigation | [`positionNavUpdate()`](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/flight/position_nav.c#L140) | Converts target position to limited target velocity. |
| Position/velocity control | [`positionControl()`](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/flight/autopilot_multirotor.c#L663) | Combines position/velocity errors and produces roll/pitch targets. |
| Attitude control | [`pidLevel()`](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/flight/pid.c#L560) | Converts selected attitude targets into rate setpoints. |
| Rate control and mixing | [`pid.c`](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/flight/pid.c) and [`mixTable()`](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/flight/mixer.c#L676) | Stabilizes the aircraft and produces motor outputs. |

Position hold calls the shared controller from
[`updatePosHold()`](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/flight/pos_hold_multirotor.c#L87).
Rescue, altitude hold, and position hold are scheduled tasks in
[fc/tasks.c](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/fc/tasks.c#L400).
Camera processing should publish an observation from a separate bounded task or
processor. The navigation task should consume the latest fresh observation;
image extraction/matching should not block the gyro/rate PID task.

## Recommended interface: an observation, with FC-owned guidance

This is a proposal based on the inspected control structure. It is not a stock
Betaflight wire protocol. The vision module should report where home is, while
the flight controller owns the final control decision:

| Proposed field | Contract |
| --- | --- |
| Protocol version, sequence, home/session ID | Detect incompatible packets, replay/out-of-order data, and stale home maps. |
| Exposure timestamp and receipt time | Convert into an agreed clock; enforce age when consumed, not only when received. |
| Home displacement | Camera-to-home horizontal error in meters, in an explicitly identified frame. Alternatively use a calibrated target ray and perform projection on the FC. |
| Image and metric validity | Explicit invalidation; absent output must not reuse the last target indefinitely. |
| Tracking quality | Inliers, support, reprojection error, and rejection reason; these are not calibrated covariance. |
| Geometry/telemetry identity | Calibration, camera mount, height above landing plane, and attitude synchronization used for projection. |

The existing Python observation supplies much of this, but clock mapping, home
identity, transport, and FC frame conversion still need implementation.

The Python local-world x/y frame is not automatically Betaflight ENU. The examined
[position estimate](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/flight/position_estimator.h)
uses ENU centimeters and cm/s. Navigation target positions use ENU meters, while
`positionNavGetTargetVelocityCmS()` returns cm/s. Camera attitude/mount rotation,
yaw convention, camera/body offset, units, and measurement time must be explicit.
Do not infer units from one comment: the autopilot header labels angles as
centidegrees while `pidLevel()` consumes `autopilotAngle` as degrees; verify the
producer/consumer expressions in the selected build.

## How to prevent controllers fighting

Add a flight-controller-owned source selector for horizontal guidance:

```text
GPS return target ───────────────┐
                                ├─ XY source selector ─ shared guidance/control
Validated visual home error ────┘                      ─ attitude ─ rate PID ─ mixer
```

Only one source supplies horizontal guidance at a time. Keep altitude/descent,
yaw, arming, touchdown, and failsafe authority explicit and owned by the FC.
Visual alignment may gate descent, but the camera module does not independently
write throttle, motor outputs, or arming channels.

Proposed states: GPS_RETURN -> VISION_ACQUIRE -> VISION_ALIGN -> VISION_DESCEND,
with a distinct VISION_LOST transition. Acquisition requires fresh consistent
observations, usable height/attitude, plausible geometry, and an approved
operating range. State transitions need hysteresis, bounded acceleration/tilt,
and deliberate treatment of controller integrals/anchors for smooth transfer.
The current Python timing thresholds are experiment settings, not validated
flight limits.

On loss, invalidate visual guidance immediately. The FC policy decides whether
to hold descent, re-acquire, return to GPS, or abort. A GPS fallback can move home
out of the narrow camera view near the ground, so that transition needs specific
tests. Pilot recovery during RX-loss failsafe must follow the actual firmware's
rules, not the simulator's immediate override shortcut.

A source selector alone is insufficient if GPS feedback still dominates final
position correction. `positionControl()` currently also anchors a navigation
command to its ENU target and subtracts the estimated position. A precision path
must consistently replace the relevant position error/anchor and velocity target
or fuse a properly modelled visual measurement. Merely adding a vision velocity
correction while leaving the GPS position error active can recreate the conflict
inside the shared controller. Likewise, constructing `GPS position + visual
error` as a target can reintroduce GPS jitter and timing errors between updates.
This is a central design issue to resolve against the selected firmware.

For the mission path, inspect `startLanding()/updateLanding()`,
`positionNavUpdate()`, and the navigation branch of `positionControl()` together.
For the legacy path, inspect `calculateTargetStep()/clearTargetStep()` plus the
same shared controller. These are integration candidates, not confirmed insertion
points for an unseen P4 fork. Prevent mission logic from overwriting a visual
landing target on its next update.

## Why MSP stick commands are a different approach

Betaflight supports `MSP_SET_RAW_RC` (command 200), handled by
[msp.c](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/msp/msp.c#L2924).
It carries receiver-channel values, not meter offsets or m/s velocity targets.
The official [companion-control guide](https://betaflight.com/docs/wiki/guides/current/MSP-Companion-Computer-Control)
distinguishes an MSP receiver from selected-channel MSP override.

This can support a separately designed companion flight-control mode, but it
is not an automatic precision-landing input to GPS Rescue. In the examined code,
`pidLevel()` selects autopilot angles during Rescue/position control;
[rc.c](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/fc/rc.c#L648)
also selects autonomous yaw, and the mixer selects autonomous throttle during
Rescue/altitude hold. RC values may be superseded or interpreted as pilot intent.
Do not run an independent stick-driving controller alongside active Rescue and
assume they will combine correctly.

The inspected 2026.6.2 MSP RC override implementation checks channel freshness
with a 300 ms window in
[rx/msp.c](https://github.com/betaflight/betaflight/blob/e0b7bb01b17b21351057e9ead2d1ab39dd44fa16/src/main/rx/msp.c#L79).
That RC timeout is not a visual observation watchdog. Version/fork behavior must
be checked separately. No RC override commands were sent in this investigation.

I did not find a remembered-home visual landing observation receiver in the
examined MSP dispatcher and v2 protocol headers. Existing GPS, rangefinder, and
optical-flow messages should not be assumed to provide this contract. A custom
receiver/API and internal source selection are likely needed; confirm against
the P4 fork before assigning a message ID or implementing it.

## Concrete next work

1. Use the identified P4 pin/worktree and legacy Rescue build above. Confirm
   the attached board matches its archived image when available, then review
   scheduler/camera availability, estimator inputs, and control units there.
2. Define the observation packet and implement an observation-only receiver
   with freshness/sequence/frame checks and debug logging. No guidance authority.
3. Implement the FC-owned XY source selector and chosen feedback strategy in a
   software/HIL test target. Exercise GPS drift, camera delay/loss, height bias,
   source transfers, stale packets, pilot override, and RX-loss failsafe.
4. Measure control stability using Betaflight control code. The current kinematic
   Python simulation does not execute its position, attitude, rate, or mixer loops.
5. Revisit hardware guidance only after the code path and tests establish exactly
   which component owns each output. Keep the existing bench connection read-only
   until a specific hardware-control test is separately authorized.
