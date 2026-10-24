# Purchased camera and integration plan

Decision recorded 2026-10-24: user bought the linked **InnoMaker OV9281
monochrome global-shutter MIPI camera**, Amazon ASIN `B09WTP5GZH`.
[Purchase listing](https://www.amazon.se/dp/B09WTP5GZH).
The listing was not directly accessible during research; confirm the exact
module revision, included cable and supplied lens when it arrives.

Target board is the **DFRobot FireBeetle 2 ESP32-P4, DFR1237**.
[DigiKey listing](https://www.digikey.se/sv/products/detail/dfrobot/DFR1237/28531124).
DFRobot documents a two-lane MIPI CSI connection with Raspberry Pi 4B camera
pin-sequence compatibility, 32 MB PSRAM and 16 MB flash. These are board
specifications, not proof that the installed Betaflight build enables PSRAM or
camera support. [Board documentation](https://wiki.dfrobot.com/dfr1237/).

## Verified capability versus unresolved compatibility

The OV9281 sensor is monochrome, global shutter, 1280 × 800 at up to 120 fps,
with two-lane MIPI and 8/10-bit RAW output.
[Sensor manufacturer](https://www.ovt.com/products/ov9281/).
Global shutter avoids rolling readout distortion; short exposure is still
needed for motion blur, with enough illumination and controlled gain.

Espressif's inspected upstream OV9281 configuration provides RAW8 modes at
640 × 400 / 100 fps and 1280 × 720 / 50 fps, requiring two lanes and a 24 MHz
input clock. These are the inspected driver modes, not a promise of 120 fps
in this firmware. Pin a compatible component/IDF version before implementation.
[Driver configuration](https://github.com/espressif/esp-video-components/blob/master/esp_cam_sensor/sensors/ov9281/Kconfig.ov9281).

Raspberry Pi connector compatibility does not install a Linux driver into
Betaflight. Check FPC contacts/orientation, CSI lanes, power rails, sensor control
bus/address, reset/power-down and module clock source against both schematics.
Verify the received board's oscillator arrangement against the driver clock
expectation. Qualify RAW8 capture and grayscale processing; do not assume a Bayer
color ISP pipeline is appropriate for a monochrome sensor.

## Lens choice and ground coverage

The starting recommendation is a low-distortion **80–90° horizontal FOV**, with
adjustable focus and preferably an interchangeable lens. This is a design
recommendation pending simulation/recording results, not a confirmed specification
of the purchased camera. Seller angles may be diagonal instead of horizontal.
An InnoMaker V2 manual lists one supplied lens as 148° diagonal / 118° horizontal
with substantial barrel distortion; confirm whether that is the delivered lens.
[Manufacturer manual](https://www.inno-maker.com/wp-content/uploads/2022/05/CAM-MIPIOV9281-V2-User-Manual-V1.2.pdf).

For an ideal rectilinear camera looking vertically at flat ground,
`ground width = 2 × camera height × tan(horizontal FOV / 2)`:

| Horizontal FOV | Width at 2 m | Width at 10 m |
| --- | --- | --- |
| 60° | 2.3 m | 11.5 m |
| 80° | 3.4 m | 16.8 m |
| 90° | 4.0 m | 20.0 m |
| 120° | 6.9 m | 34.6 m |

Wider coverage helps retain overlap and find home despite return error/tilt;
at a fixed resolution it gives fewer pixels per meter. Calibration, distortion
correction/cropping and tilt change usable coverage. A visible home point alone
does not guarantee enough matching ground support. Calibrate the delivered lens
at the actual focus, resolution and crop; secure focus against vibration.

The simulator still uses 640 × 480 with fx=fy=500 pixels (about 65.2° horizontal
FOV), not a measured OV9281 profile. FOV sweeps and 640 × 400 / 1280 × 800 sensor
profiles have been proposed but are not yet implemented as user-selectable modes.

## Bring-up and performance gates

First obtain reliable timestamped frames without recognition. Then establish
intrinsics/distortion, rigid camera mounting rotation/offset and exposure-time
alignment with attitude/height. Record real ground sequences for replay.

Capture cadence and vision update cadence are separate. Process fresh frames
with bounded buffers; discard obsolete work rather than accumulating delay.
An initial native experiment might use 100–200 features at 5–10 processing Hz
on reduced grayscale images; quality and worst-case cost must be measured.
Do not assume all 100/120 captured frames per second can or should run ORB.

Measure control deadlines/jitter, receiver service, frame age, dropped frames,
per-stage processing, DMA/PSRAM bandwidth, stack and total memory with camera off,
capture only, and capture plus vision. No onboard vision throughput has been
measured. Integration sequencing and ownership rules are in
[vision architecture](vision_architecture.md) and [project status](project_status.md).
