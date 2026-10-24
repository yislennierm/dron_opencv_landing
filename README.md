# Drone OpenCV Landing

Experimental Python/OpenCV demos for visual return-to-home (RTH) and landing
guidance. The camera records reference views of the ground, then matches a live
view against them to locate a stored home point.

The pipeline uses FAST corner detection, ORB descriptors, Hamming-distance
matching, and RANSAC homography estimation. The webcam demos display the home
point and its offset from the image center. Optional altitude values and camera
field of view provide an approximate conversion from pixels to meters.

This is a webcam research prototype. Flight-controller integration is not
implemented. Reference views are kept in memory and are lost when a demo exits.

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

## Scripts

| Script | Purpose |
| --- | --- |
| `FAST_corner.py` | Match an image against a synthetically transformed copy and save a visualization. Defaults to a generated checkerboard. |
| `fast_orb_profiler.py` | Measure time spent loading, detecting, describing, matching, estimating alignment, and visualizing. |
| `webcam_rth_demo.py` | Basic manual reference capture and landing guidance. |
| `webcam_rth_demo2.py` | Adds camera selection and raw/IR capture options. |
| `webcam_rth_demo3.py` | Adds selection of a home point and attempts to carry it across reference views. |
| `webcam_rth_auto_levels.py` | Attempts automatic reference capture during takeoff and reference selection during landing. |
| `webcam_rth_auto_levels1.py` | Variant with modified scale estimation and capture thresholds. |

The numbered files are experimental variants, not a tested release sequence.
Image alignment and home-point propagation still need validation before relying
on their output.

## Run

Start with the manual webcam demo:

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
