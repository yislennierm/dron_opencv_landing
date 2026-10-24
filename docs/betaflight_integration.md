# Standard Betaflight: visual landing interface and commands

Reviewed 2026-10-24. Scope: a normal Betaflight flight controller connected to
a vision processor. The intended final design is a built-in subsystem, described
in [the vision architecture proposal](vision_architecture.md). The external
USB/UART path below is a development/companion option; an onboard camera backend
can publish the same measurements internally without UART. This review does not
implement an ESP32-P4 port.
The earlier P4 investigation is retained separately in
[historical source notes](betaflight_p4_source_notes.md).

## Finding and recommendation

Use **MSP over USB for the first bench connection**, and MSP over a dedicated
hardware UART for an onboard companion when the board's port/electrical details
are known. The transport exists today. A remembered-home visual landing
observation API into GPS Rescue does not exist in the inspected stock handlers.

Two different integration choices must not be confused:

| Approach | Existing interface | What we must implement |
| --- | --- | --- |
| Stock firmware with companion steering | `MSP_SET_RAW_RC` through configured MSP receiver/override | Companion closed-loop control, exclusive mode/authority handoff, loss handling, and conversion into RC values. This does not add a visual input to Rescue. |
| Vision assisting Betaflight's RTH/final landing | Proposed custom MSP v2 observation + receiver/status API | A small firmware extension, a validated observation mailbox, and FC-owned horizontal-guidance source selection with matching feedback/anchor policy. |

For this project's intended GPS-return-then-precision-landing feature, the second
approach is the recommended design. The recommendation is an architectural
inference from the inspected control paths, not a claim that the custom API
already exists. No write/control packets or configuration changes were made.

## Source baseline

The local stock checkout is `/home/nene/Documents/betaflight`, official upstream
remote, commit `1c6716a886be8e9625faad0b27aca6b7af972c4e`. This review inspects its
actual command handlers, receiver override, telemetry and control code. Its
existing `.devcontainer/devcontainer.json` edit was left untouched. The boards
previously inspected over USB reported 2026.6.2 / API 1.48; the exact flashed
build still needs runtime identification before implementing a device adapter.
The local checkout and generated online documentation must not be treated as
guaranteed descriptions of every released or custom binary.

The subsequently connected FireBeetle is a different build: `FP4B` /
`FIREBEETLE2P4_BENCH`, `2026.12.0-alpha`, API 1.49, revision `e5071ce`.
Its matching source has not been found. See [current project status](project_status.md)
for this hardware identity and the distinction from the older local P4 source.

Primary references:

- [Official serial interface guide](https://betaflight.com/docs/wiki/guides/current/Serial)
- [Official MSP companion-control guide](https://betaflight.com/docs/wiki/guides/current/MSP-Companion-Computer-Control)
- [Generated MSP reference](https://betaflight.com/docs/development/MSP-Protocol-Reference-Dev)
- [Pinned stock MSP dispatcher](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/msp/msp.c)

The generated reference itself names a different source commit. The pinned
handler implementation, plus the connected board's API/build, takes precedence
for payload parsing and command support. A symbol in a protocol header alone is
not proof that firmware implements it.

## Physical connection and protocol

| Connection | Use | Requirements |
| --- | --- | --- |
| USB serial / VCP | Computer-based bench vision and diagnostics | Data-capable USB connection, serial access, board supporting the interface; request/reply MSP. |
| Hardware UART | Onboard companion connection | Enable MSP on a free UART, match baud rate, connect companion TX to FC RX and companion RX to FC TX with common ground. Verify board logic levels (commonly 3.3 V TTL) and port/resource sharing. |
| MAVLink serial | Alternative receiver/telemetry/GCS integration | Build- and version-dependent handlers; not an automatic ArduPilot/PX4-style offboard or precision-landing API. |

MSP v1 and v2 can use the same bidirectional serial transport. Use request/reply
telemetry initially; unsolicited telemetry push over USB is build-dependent.
115200 baud is a reasonable initial UART setting supported by the stock MSP
configuration, but actual port capacity and scheduling must be measured. USB
VCP does not consume a physical UART; its line-setting baud value does not
necessarily limit USB throughput as a UART baud rate would.

Avoid hardcoding CLI port-assignment commands across versions: the official
serial guide documents the change from per-port function masks to per-feature
settings in newer firmware. Selecting a port is a separate action from enabling
or selecting a sensor/protocol. This review changes neither.

## Existing read requests needed by the vision adapter

These are MSP v1 command IDs. Availability and optional payload tails must be
checked against the device/API; use little-endian decoding matching the handler.

| Request | ID | Purpose / important interpretation |
| --- | --- | --- |
| `MSP_API_VERSION` | 1 | Negotiate API parsing. |
| `MSP_FC_VARIANT` / `MSP_FC_VERSION` | 2 / 3 | Establish firmware identity/version. |
| `MSP_BOARD_INFO` / `MSP_BUILD_INFO` | 4 / 5 | Identify board and build revision. |
| `MSP_STATUS` / `MSP_STATUS_EX` | 101 / 150 | Sensors, active modes, arming blocks and other runtime status. |
| `MSP_RC` | 105 | Read effective receiver channels; useful for diagnostics, not a control-authority grant. |
| `MSP_RAW_GPS` | 106 | Fix, satellites and GPS navigation measurements. |
| `MSP_COMP_GPS` | 107 | GPS distance/bearing to home; not the visual home error. |
| `MSP_ATTITUDE` | 108 | Roll/pitch in signed tenths of a degree; heading in whole degrees in the examined implementation. |
| `MSP_ATTITUDE_QUATERNION` | 167 | Four signed int16 components scaled by 32767 in the examined handler, not four float32 values. Probe support. |
| `MSP_ALTITUDE` | 109 | Estimated altitude in centimeters plus optional/available vertical-speed fields. This is not automatically camera height above the landing plane. |
| `MSP_BOXIDS` | 119 | Interpret packed active-mode bits using this build's active mode list. |

See [the pinned response handlers](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/msp/msp.c#L1394)
and [protocol identifiers](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/msp/msp_protocol.h).

Most of these responses do not provide the camera's exposure timestamp or a
synchronized FC attitude sample at that exposure. Polling attitude and attaching
host receipt time is a bench approximation. Real geometry needs camera mounting
calibration, true height above the ground plane, clock mapping and attitude
history/interpolation or another explicitly synchronized acquisition strategy.

## Existing write commands: what they really mean

### MSP_SET_RAW_RC — 200

This is the stock external steering input. Payload: a positional sequence of
little-endian uint16 RC channel values, typically in a 1000–2000 microsecond
representation. Roll/pitch/yaw center is normally 1500; that does not establish
hover throttle. Channel order/mapping is configuration-dependent. The inspected
handler accepts up to its configured maximum supported channel count.

It can supply all RC through `RX_MSP`, or substitute only masked channels when
a real receiver and the `MSP OVERRIDE` mode are configured. In the latter case,
ARM and the takeover switch should remain receiver-owned channels outside the
companion's mask. The exact mask, selected mode and receiver-loss policy must be
read back, not inferred from the sender's intentions.

The official companion guide recommends a steady 25–50 Hz RC stream. In the
examined receiver implementation, each overridden channel needs a fresh frame
containing that channel within 300 ms. Older releases can behave differently,
including holding the last values. This RC freshness check is not a vision-age
watchdog or a substitute for handling tracking loss.

Sources: [input handler](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/msp/msp.c#L3052),
[override channel selection](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/rx/msp_override.c#L30),
[freshness logic](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/rx/msp.c#L79).

It is not a velocity command: sending `0.4` for "move right at 0.4 m/s" has no
meaning to this interface. An external position/velocity controller would have
to generate RC inputs appropriate to the selected flight mode. Active Rescue
can override attitude/yaw/throttle, so companion stick control and active Rescue
must not be assumed to combine into precision guidance.

### Sensor injection: existing but different measurements

The examined MSP dispatcher handles:

| Message | ID | Input meaning |
| --- | --- | --- |
| `MSP_SET_RAW_GPS` | 201 | GPS-like fix/position/speed input. |
| `MSP2_SENSOR_GPS` | `0x1F03` | Richer GPS solution input. |
| `MSP2_SENSOR_RANGEFINDER_LIDARMT` | `0x1F01` | Supported rangefinder packet; compiled sensor driver/configuration required. |
| `MSP2_SENSOR_OPTICALFLOW_MT` | `0x1F02` | Supported MicoAir optical-flow packet; driver-specific motion scale, timing and range requirements. |

[Sensor dispatch](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/msp/msp.c#L4110)
and [range/flow decoding](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/drivers/rangefinder/rangefinder_lidarmt.c#L175)
show why these are not interchangeable with home displacement. Optical flow
measures image motion, while our tracker identifies a remembered physical home.
Recasting visual home error as GPS position would replace navigation measurements
and complicate the GPS fallback rather than provide a dedicated landing input.

The header also defines `MSP_SET_WP` (209) and `MSP_SET_HEADING` (211), but no
handlers for those names were found in this stock source revision. Do not select
a navigation API based on these legacy identifiers. `MSP_SET_MOTOR` (214) is a
motor-test interface and is not a landing-guidance interface.

### MAVLink does not imply precision-landing support

This checkout includes RC_CHANNELS_OVERRIDE receiver handling, telemetry, and
feature-gated GCS commands/mission support. Its GCS command executor recognizes
arm/disarm, mode selection, return-to-launch, setting home, and reboot/shutdown.
That does not imply support for every MAVLink message. In the inspected handlers,
I found no `LANDING_TARGET`, `SET_POSITION_TARGET_LOCAL_NED`, `SET_ATTITUDE_TARGET`,
`VISION_POSITION_ESTIMATE`, or `ODOMETRY` input path for our purpose.

See [command/message dispatch](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/telemetry/mavlink.c#L670),
[RC receiver](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/rx/mavlink.c#L110),
and [the official MAVLink setup guide](https://betaflight.com/docs/wiki/guides/current/MAVLinkELRS).

## Proposed dedicated observation interface

For GPS-assisted precision landing, retain MSP transport and introduce versioned
MSP v2 messages with new firmware handlers. These are proposed names, not existing
commands; numeric IDs have not been allocated:

- `VISION_HOME_OBSERVATION`: submit one observation or explicit invalidation.
- `VISION_LANDING_STATUS`: read accepted sequence, observation age/rejection,
  active guidance source and landing state. An MSP ACK means parsed/received;
  it must not be confused with accepted guidance authority.

Proposed observation contract:

| Field | Purpose |
| --- | --- |
| Protocol version / packet length | Reject incompatible layouts. |
| Sequence and home/session ID | Reject stale, out-of-order or wrong-home observations. |
| Exposure time in an agreed clock | Enforce freshness when guidance consumes the observation. |
| Frame identifier | State exactly which coordinates the error uses. |
| Horizontal camera-to-home error in ENU meters | Existing vision output after explicit transformation into the FC navigation frame. |
| Image validity and metric validity | Never retain an old valid correction after a new invalid observation. |
| Quality fields | Inliers, support and reprojection error; not invented covariance/confidence. |

Exact binary types, clock synchronization, quality gates, delivery rate, timeout
and ID allocation remain design/implementation work. The Python local-world x/y
frame is not automatically FC ENU. If projection is moved onto the FC, a calibrated
target ray can be used instead, but that is a different negotiated payload.
A structured parser/serializer must be tested before any device write is added.

## Controller-side placement and authority

The serial handler should validate framing/length and publish a coherent,
bounded latest observation. It should not run the image pipeline, write motor
outputs, change arming, or directly execute control from the serial callback.

The scheduled navigation/controller code then selects one horizontal error/target
source: GPS return, visual approach, or an explicit fallback. Betaflight's attitude,
rate and mixer loops remain the only downstream stabilizing/control chain.
Altitude and yaw ownership must also be explicit. The low-rate camera pipeline
must not block the gyro/rate loop.

The firmware's shared control path currently uses both position error/anchor and
velocity targets. A visual velocity correction cannot simply be added while
leaving an independently competing GPS position anchor active. The selected
source must cover the relevant feedback and integral/anchor transfer policy.

Concrete source locations in the stock checkout:

- [Rescue target generation](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/flight/gps_rescue_multirotor.c#L270)
- [Shared horizontal mode selection](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/flight/autopilot_multirotor.c#L834)
- [Shared position controller](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/flight/autopilot_multirotor.c#L981)
- [Autonomous attitude selection](https://github.com/betaflight/betaflight/blob/1c6716a886be8e9625faad0b27aca6b7af972c4e/src/main/flight/pid.c#L587)

The compiled version may use legacy Rescue or Rescue-as-mission. Target the actual
build before adding a hook; the correct interface must prevent either path from
overwriting visual guidance on its next update. Visual loss must have a defined
transition and cannot simply leave the last correction in effect. Pilot takeover
and RX-loss failsafe remain subject to Betaflight's actual policy, not the
simulator's immediate takeover shortcut.

## Implementation order

1. A read-only USB MSP adapter: identify firmware, request status/attitude/GPS,
   decode active modes, measure round-trip latency, and handle disconnects.
2. Define/test the dedicated observation packet and receiver-status contract;
   drive it from simulation observations with delay/loss/out-of-order cases.
3. Add a logging-only receiver in a selected stock Betaflight software test build.
   Confirm parsed vs accepted vs active-source states without steering hardware.
4. Add and test the FC-owned source selector/feedback integration, including
   controller transfer, GPS drift, telemetry/height errors and tracking loss.
5. Evaluate the actual Betaflight controllers in software/HIL before hardware
   guidance tests. The existing Python simulator is kinematic and does not execute
   the Betaflight control loops.

Existing authorization for hardware remains read-only: no RC overrides, arming,
motor tests or configuration writes are part of this investigation.
