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

See [geometry and validation](docs/landing_baseline.md) and the
[commercial drone comparison](docs/commercial_drones.md).

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
