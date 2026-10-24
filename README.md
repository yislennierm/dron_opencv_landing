# Drone OpenCV Landing

Python/OpenCV proof of concept for landing over a remembered natural takeoff
area, with no printed marker. The eventual target is an ESP32-P4-based flight
controller; the current milestone validates the vision and geometry in Python.

The `landing` package is the maintained baseline. It uses oriented, multiscale
ORB features, mutual descriptor matching, and reference-to-live homographies.
It includes bounded reference storage, alignment quality checks, temporal lock
confirmation, calibrated ground projection, and replay with ground-truth metrics.

The output is a timestamped observation with separate image-tracking and metric
validity. Metric output requires calibration, current camera height above the
ground, and synchronized camera attitude. Reference views live in memory; replay
manifests reproduce them. Motor control and flight-controller integration remain
future work.

Start with [project status and decisions](docs/project_status.md) for the current
implemented features, hardware/source identities, known limits and next milestones.
The purchased InnoMaker OV9281 and planned FireBeetle camera bring-up are recorded
in [the camera plan](docs/camera_plan.md); onboard capture/vision is not implemented.

See [geometry and validation](docs/landing_baseline.md), the
[commercial drone comparison](docs/commercial_drones.md), and the
[Betaflight integration source review](docs/betaflight_integration.md). The
[intended built-in vision architecture](docs/vision_architecture.md) separates
camera/board configuration, local motion estimation, remembered-home landing,
sensor fusion, and flight-control authority; it remains a design proposal.

## Setup

Use Python 3.11 or 3.12 as a starting point for a new environment. A graphical
desktop and camera access are required for the webcam demos. Dependencies are
pinned to the versions found in the original development environment; a fresh
installation on other platforms has not yet been verified.

```sh
git clone git@github.com:yislennierm/dron_opencv_landing.git
cd dron_opencv_landing
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

On Windows, use `py -m venv .venv` and activate with
`.venv\Scripts\Activate.ps1` in PowerShell. GitHub access to the private repository
must be configured on each computer. HTTPS cloning is also available:
`https://github.com/yislennierm/dron_opencv_landing.git`.

## Run the maintained baseline

### Explore landing without hardware

```sh
python -m landing simulate
```

This opens a local, offline 3D simulator in your browser. **Take off** captures
home and ascends to the selected altitude, then waits in manual flight. Hold the
cross buttons (or keyboard arrow keys) to move in camera-view directions. Release
to brake. **Climb / Descend** buttons or **W / S** control height, bounded between
0.5 and 12 m; descending to 0.5 m lands at the current location. The center
**Brake** button takes manual control and requests zero velocity.

Press **Return home** or **R** at any airborne stage. When below the selected
takeoff altitude, the quad first climbs while holding horizontal position, then
uses simulated GPS and actual `LandingTracker` ORB matches/calibrated offsets
for final approach. Saved home references and the flight trail are preserved.
Return does not guarantee visual acquisition if mapping or visibility fails.
Holding a movement control during return takes manual control again. Return also
works while paused and resumes simulation. Inputs clear on release, focus loss,
pause, reset, and return. Keyboard flight shortcuts yield to focused sliders
and tabs so those controls remain accessible.

The **Auto demo** button in the Flight tab retains the original automatic
ascent/outbound/return sequence, using the selected outbound distance. In manual
flight, outbound distance only sets the initial spectator view, not a movement
target. The spectator view zooms out as you travel farther from home. Flight
controls affect only this software simulation; there is no hardware connection.

Drag to orbit and scroll to zoom. Before starting, set takeoff altitude (2–12 m),
outbound distance (2–100 m), return heading (−180° to 180°), and maximum camera
tilt (0–15°). The browser defaults to a 6 m ascent, 20 m outbound flight, 90°
return heading, and up to 8° tilt. These plan settings are fixed during a flight;
reset to edit them. Adjust GPS home error and wind live, or obscure the ground
to explore tracking loss. Disable visual guidance to compare
GPS-only landing. Reset or start another flight to compare settings. The camera
panel shows the processed frame with the detected home overlay. The controls fit
the viewport using Flight, Live, Status, and About tabs; the camera, run buttons,
and tracking summary stay visible. Status contains telemetry and landing error.
Switch tabs with a click or the left/right arrow keys when a tab has focus.

The on-screen **Flight controls / requested movement** panel shows the exact velocity targets used
by the simulator, with the active source labeled `PILOT`, `VISION`, `GPS`, or
`FLIGHT PLAN`.
Its up/down/left/right arrows describe horizontal movement relative to the
current camera view; up/down there does not mean altitude change. Ascend/descend
is shown separately and identifies pilot or simulator control. Values are requested
velocities in m/s, not measured velocities, RC stick positions, or motor commands.
Arrows highlight above a display-only threshold of 0.02 m/s. On pause or connection
failure, the panel labels values as the last setpoint and clears active arrows.

The tracker itself outputs a timestamped home position offset, tracking validity,
and quality. The panel also shows its raw image offset in pixels and calibrated
world offset in meters. The simulator's proportional guidance converts the world
offset to a velocity target during visual approach; takeoff, outbound flight, GPS
return, descent decisions, wind, and motion are simulated. This display does not
add flight-controller or motor control to the tracker. With vision disabled or
lost, the source changes to simulated GPS; invalid visual offsets are not reused.

The **Vision** selector above the camera exposes the processing stages:

- **Camera input**: the rendered color frame.
- **Grayscale / rectified**: the actual preprocessed tracker input.
- **ORB features**: the selected feature points from that processing pass.
- **Saved reference**: the selected reference (or a labeled candidate/latest view)
  and its saved home anchor.
- **Descriptor matches**: saved reference on the left, live view on the right;
  green lines are RANSAC inliers and red lines are other descriptor matches.
- **RANSAC inliers**: consensus matches before final validation; these alone do
  not establish tracking lock.
- **Alignment / support**: the projected reference boundary and live inlier hull,
  with the geometry decision and final observation status.
- **Confirmed home**: the center-to-home arrow and cross, only for valid output.

Click **⤢** to enlarge a stage; the enlarged view also has a stage selector and
Pause/Resume. Switching views while paused or landed does not advance simulation
or rerun the vision pipeline. Diagnostic overlays are applied after processing.
Side-by-side match views show at most 120 evenly sampled links for readability;
counts report every match. When no reference is selected, an explicitly labeled
candidate with strongest geometric support/most matches is shown. Rejected
candidates and RANSAC consensus are not presented as confirmed home estimates.

The simulator enables optional tracker debug capture. It retains the current
processed grayscale frame, selected features, actual descriptor correspondences,
RANSAC mask/homography and decisions for each attempted reference, plus one
processed grayscale image per saved reference (up to 12 × 640 × 480 bytes with
simulator defaults). This adds diagnostic memory beyond `reference_bytes`.
Debug capture is off by default for normal webcam/replay use; images are never
added to the normal reference bank. Rendering/encoding diagnostics is outside
the reported tracker latency, while debug capture inside the tracker is included.

The default ground is a procedurally generated **256 × 256 m gravel/soil plane**.
Nearby terrain is generated on demand at 192 pixels/m, retaining the same
close-up detail without allocating a giant image. Coordinate-seeded chunks have
continuous soil patches and stones across their boundaries, with no repeating
terrain tiles or added landing markers. The shared cache holds at most 48 chunks
(144 MiB of RGB pixels, excluding rendering/generation working memory).
Zoom out to see the larger ground boundary. Expand it further, up to 4096 m wide:

```sh
python -m landing simulate --ground-size 1024
```

The default seed is 7; choose another reproducible terrain with:

```sh
python -m landing simulate --seed 42
```

You can still use your own textured ground photograph (mapped onto a flat plane
10 m wide) with:

```sh
python -m landing simulate --image ground.jpg
```

No extra dependencies or internet connection are needed. Ctrl+C stops the local
server. Use `--no-browser` to print the URL without opening it, or `--port 8766`
to use another port. Each browser tab has a separate simulation; reload starts
a new session. The server supports up to 16 sessions before a restart.

This is a closed-loop vision test with simplified motion and ideal synchronized
height/attitude. The camera is rigidly mounted downward on the quad. Yaw turns
at up to 40°/s toward the selected return heading, while roll/pitch follow
horizontal velocity with a smoothed tilt bounded by the chosen angle per axis.
Forward movement lowers the nose and sideways movement banks toward travel.
Simulator body axes are x forward, y left, z up; positive simulator pitch lowers
the nose. The cross buttons remain camera-relative, and these angle conventions
must be converted explicitly for Betaflight. Tilt follows velocity rather than
acceleration, so physical braking attitude is not modeled.
The same rotation drives the perspective-rendered image, tracker telemetry,
quad model, and camera footprint. This is a prescribed attitude model, not an
attitude controller or aerodynamic simulation. Truth renders the camera and scores the
flight; visual horizontal guidance consumes only tracker offsets. Mapping uses
the normal bounded reference bank during ascent. Images without accepted visual
observations cannot authorize descent in vision mode. GPS error beyond the
camera's coverage or failed mapping may leave the quad waiting for acquisition.
At low height, reverting to GPS horizontal guidance after visual loss may move
home outside the camera view; recovery is not guaranteed. Touchdown is modeled
at 0.5 m camera height. Higher ascents may exhaust the normal 12-reference bank;
`reference_limit` is reported rather than silently increasing its capacity.
Simulation time advances in fixed substeps with processed frames; playback may
run slower than real time. The wind setting is an uncalibrated disturbance.

The flat texture does not model terrain relief, rolling shutter, vibration, or
real sensor timing. Generated stones are visual details, not 3D geometry, so
simulated accuracy is not real landing accuracy. The home
cross is an overlay added after tracking, not a physical marker. This simulator
does not execute Betaflight or connect to hardware. The existing `landing replay`
command separately supports recorded videos with timestamped manifests; this
interactive mode generates new camera views in response to steering.

Run repeatable trials without a browser:

```sh
python -m landing sim-trials --seeds 7,42 --output results/simulation_trials.json
```

For each terrain seed, this tests a baseline return, a longer return with 90°
yaw and tilt under stronger wind, a 1.2 s visual interruption, a GPS-only landing,
and ground obscured for the entire return. `--max-time 120` sets the simulated
per-flight deadline. Timeouts remain failures with null landing error; they are
not assigned zero error. Each run reports the flight plan, final phase, landing
error, reference count/mapping status, first return lock time, tracking recovery,
false locks on frames where home is outside the camera or obscured, and accepted
metric estimates more than 15 cm from truth (`inaccurate_locks`). The tracker is
still evaluated when visual guidance is disabled, but those estimates do not
steer the GPS-only flight. Scores use simulation truth only for evaluation.

The suite intentionally includes unsuccessful cases: GPS-only may land outside
the 15 cm tolerance, and an entirely obscured return should time out without
visual acquisition. Process exit success means the report was generated, not
that all landings succeeded. Inspect per-scenario outcomes as well as aggregate
counts. These controlled trials do not validate real flight accuracy.

### Run camera tracking

```sh
python -m landing webcam --device 0
```

Keep the camera pointed at textured, approximately flat ground. Start with a
level camera over the intended home point:

- **Space** starts or pauses mapping, setting the first home at the processed
  image center. New references are captured after validated scale changes.
- **F** forces a reference capture when alignment is good.
- **L** switches to locating home against the saved references.
- **R** resets; **Q / Escape** exits.

The overlay draws home only after three consistent observations. With no
calibration and telemetry, output is in processed pixels. For a controlled,
stationary bench setup, supply your own calibrated camera JSON and explicitly
assume a level camera and fixed current height:

```sh
python -m landing webcam --camera my_camera.json --assume-level --height 2.0 --log observations.jsonl
```

`examples/camera.example.json` documents the format; its numbers are illustrative,
not a calibration. An actual moving-camera experiment should supply per-frame
attitude and height through replay or the Python API. Height is measured from the
camera to the ground plane, not barometric altitude above takeoff.

Run repeatable checks without a camera or GUI:

```sh
python -m unittest discover -s tests -v
python -m landing benchmark --check --output results/synthetic.json
python -m landing benchmark --check --image sample.jpg --output results/sample.json
python -m landing replay examples/sample_replay.json --output results/replay.json
```

The sample replay reuses a single image to check the replay interface; it is not
an ascent/descent recording.
Benchmarks render known planar transformations and report error, rejected frames,
false locks, recovery frames, and processing latency. Synthetic accuracy does not
establish real landing accuracy or ESP32-P4 throughput.

## Original experimental scripts

| Script | Purpose |
| --- | --- |
| `FAST_corner.py` | Match an image against a synthetically transformed copy and save a visualization. Defaults to a generated checkerboard. |
| `fast_orb_profiler.py` | Measure time spent loading, detecting, describing, matching, estimating alignment, and visualizing. |
| `webcam_rth_demo.py` | Basic manual reference capture and landing guidance. |
| `webcam_rth_demo2.py` | Adds camera selection and raw/IR capture options. |
| `webcam_rth_demo3.py` | Adds selection of a home point and attempts to carry it across reference views. |
| `webcam_rth_auto_levels.py` | Attempts automatic reference capture during takeoff and reference selection during landing. |
| `webcam_rth_auto_levels1.py` | Variant with modified scale estimation and capture thresholds. |

These files preserve the original experiments, including known direction, scale,
and descriptor-orientation problems. Use `python -m landing` for the corrected
baseline. The numbered files are not a tested release sequence.

## Running the original experiments

The original manual webcam demo:

```sh
python webcam_rth_demo3.py --device 0 --resolution 640x480
```

- Click a point and press **H** to choose home, or press **H** to use the center.
- Press **Space** to capture each reference view in MAP mode.
- Press **L** to enter LAND mode, then **Space** to step to a lower reference.
- Press **M** to return to mapping, **R** to reset, or **Q / Escape** to quit.

For the automatic reference-capture experiment:

```sh
python webcam_rth_auto_levels1.py --device 0
```

Press **Space** to start/pause takeoff mapping, **L** to start automatic reference
selection for landing, **M** to stop landing, and **R** to reset. These keys change
the visual demo state; they do not operate a drone.

Run an image-only pipeline check without a camera or GUI:

```sh
python fast_orb_profiler.py --image sample.jpg --target-mp 0.6 --iterations 3 --no-viz
```

Each script supports `--help` for its full options. Optional BRIEF experiments
in the profiler require `opencv-contrib-python` in place of `opencv-python`.

## Work across computers and collaborate

Before starting work, update your checkout. Keep changes on a branch and open a
pull request on GitHub:

```sh
git switch main
git pull --ff-only
git switch -c describe-your-change
# Edit and check your changes.
git add <changed-files>
git commit -m "Describe the change"
git push -u origin describe-your-change
```

Commit and push work before moving to another computer. Clone the repository
once on each computer and create a separate virtual environment there. The local
environment and generated match images are excluded from Git. Repository owners
can invite collaborators through GitHub Settings > Collaborators.
