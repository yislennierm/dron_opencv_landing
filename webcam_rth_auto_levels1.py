#!/usr/bin/env python3
import argparse
import time
from typing import List, Tuple, Optional

import numpy as np
import cv2

# ---------------- Camera helpers ----------------

BACKENDS = {
    "auto": 0,
    "avfoundation": cv2.CAP_AVFOUNDATION,
    "msmf": cv2.CAP_MSMF,
    "dshow": cv2.CAP_DSHOW,
    "v4l2": cv2.CAP_V4L2,
}

def parse_resolution(res: str):
    try:
        w, h = map(int, res.lower().split('x'))
        return w, h
    except Exception:
        return 640, 480

def try_open(device: int, backend: int, width: int, height: int, fourcc: Optional[str], convert_rgb: Optional[bool]):
    cap = cv2.VideoCapture(device, backend)
    if not cap.isOpened():
        return None
    if width > 0 and height > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if convert_rgb is not None:
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 1 if convert_rgb else 0)
    if fourcc:
        f = cv2.VideoWriter_fourcc(*fourcc)
        cap.set(cv2.CAP_PROP_FOURCC, f)
    ok, _ = cap.read()
    if not ok:
        cap.release()
        return None
    return cap

def list_cameras(max_devices: int, backend: int, width: int, height: int):
    print("[info] Scanning cameras...")
    found = []
    for i in range(max_devices):
        cap = try_open(i, backend, width, height, None, None)
        if cap:
            ok, frame = cap.read()
            if ok and frame is not None:
                shape = frame.shape
                print(f"  #{i}: opened, frame shape={shape}, dtype={frame.dtype}")
                found.append(i)
            else:
                print(f"  #{i}: opened but no frame")
            cap.release()
        else:
            print(f"  #{i}: unavailable")
    if not found:
        print("[info] No cameras found.")
    return found

def open_camera_or_exit(args):
    backend = BACKENDS.get(args.backend.lower(), 0)
    if args.list:
        list_cameras(args.max_devices, backend, *parse_resolution(args.resolution))
        raise SystemExit(0)
    cap = try_open(args.device, backend, *parse_resolution(args.resolution), args.fourcc, not args.raw)
    if not cap:
        print(f"[error] Could not open device {args.device} (backend={args.backend}). Try --list and different --device/--backend/--fourcc.")
        raise SystemExit(1)
    return cap, backend

def next_device(cur: int, step: int, max_devices: int):
    nd = cur + step
    if nd < 0:
        nd = max_devices - 1
    if nd >= max_devices:
        nd = 0
    return nd

# --------- Feature pipeline (FAST + ORB) -----------------

def detect_fast(gray: np.ndarray, threshold: int, max_kpts: int, grid=(20, 15)) -> List[cv2.KeyPoint]:
    fast = cv2.FastFeatureDetector_create(threshold=threshold, nonmaxSuppression=True,
                                          type=cv2.FAST_FEATURE_DETECTOR_TYPE_9_16)
    kps = fast.detect(gray, None)
    # spread with grid
    h, w = gray.shape[:2]
    gx, gy = grid
    cell_w, cell_h = w / gx, h / gy
    buckets = [[[] for _ in range(gx)] for _ in range(gy)]
    for kp in kps:
        cx = min(gx-1, max(0, int(kp.pt[0] / cell_w)))
        cy = min(gy-1, max(0, int(kp.pt[1] / cell_h)))
        buckets[cy][cx].append(kp)
    selected = []
    for y in range(gy):
        for x in range(gx):
            cell = buckets[y][x]
            if not cell: continue
            cell.sort(key=lambda k: k.response, reverse=True)
            selected.extend(cell[:3])
    selected.sort(key=lambda k: k.response, reverse=True)
    return selected[:max_kpts]

def compute_orb(gray: np.ndarray, kps: List[cv2.KeyPoint]):
    orb = cv2.ORB_create(nfeatures=max(2000, len(kps)), edgeThreshold=19, patchSize=31, fastThreshold=0)
    return orb.compute(gray, kps)

def match_knn(desc1, desc2, ratio=0.75):
    if desc1 is None or desc2 is None or len(desc1)==0 or len(desc2)==0:
        return []
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    raw = bf.knnMatch(desc1, desc2, k=2)
    good = []
    for pair in raw:
        if len(pair) < 2: continue
        m, n = pair
        if m.distance < ratio * n.distance:
            good.append(m)
    return good

def estimate_h(kps1, kps2, matches, reproj_thresh=2.0):
    if len(matches) < 4: return None, None
    pts1 = np.float32([kps1[m.queryIdx].pt for m in matches])
    pts2 = np.float32([kps2[m.trainIdx].pt for m in matches])
    H, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, reproj_thresh)
    inliers = mask.ravel().tolist() if mask is not None else None
    return H, inliers

def px_to_m(px: float, img_w: int, fov_deg: float, Z_m: float) -> Optional[float]:
    if fov_deg <= 1e-3 or Z_m is None or Z_m <= 0:
        return None
    fov_rad = np.deg2rad(fov_deg)
    ground_width = 2.0 * Z_m * np.tan(fov_rad / 2.0)
    return px * (ground_width / float(img_w))

def decompose_scale(H: np.ndarray, pts1=None, pts2=None) -> float:
    """
    Estimate relative scale between two images from homography H.
    Methods combined:
      1) Geometric mean of singular values of the 2x2 linear part (sqrt(det(A))). Robust to rotation.
      2) (Optional) Median pairwise distance ratio from matches (if pts provided).
    Returns a single robust scale estimate.
    """
    A = H[:2, :2]
    detA = float(A[0,0]*A[1,1] - A[0,1]*A[1,0])
    s_geo = np.sqrt(abs(detA)) if detA != 0 else 1.0
    candidates = [s_geo]
    if pts1 is not None and pts2 is not None and len(pts1) >= 8:
        # Robust median distance ratio over random pairs (limit to 200 pairs)
        idx = np.random.choice(len(pts1), size=min(len(pts1), 200), replace=False)
        P1 = pts1[idx]; P2 = pts2[idx]
        ratios = []
        for i in range(0, len(P1)-1, 2):
            d1 = np.linalg.norm(P1[i] - P1[i+1])
            d2 = np.linalg.norm(P2[i] - P2[i+1])
            if d1 > 1e-3 and d2 > 1e-3:
                ratios.append(d2/d1)
        if ratios:
            candidates.append(float(np.median(ratios)))
    return float(np.median(candidates)) if candidates else 1.0

class Level:
    def __init__(self, img_gray, kps, desc, alt_m: Optional[float], home_pt: Tuple[float,float], scale_from_lvl0: float):
        self.img_gray = img_gray
        self.kps = kps
        self.desc = desc
        self.alt_m = alt_m
        self.home_pt = home_pt
        self.scale_from_lvl0 = scale_from_lvl0  # cumulative scale vs level 0

# ---------------- Main app ----------------------

def run(args):
    cap, backend = open_camera_or_exit(args)
    device = args.device
    width, height = parse_resolution(args.resolution)
    alt_list = [float(x) for x in args.alts.split(',') if x.strip()] if args.alts else None

    # States
    IDLE, TAKEOFF, LAND = "IDLE", "TAKEOFF", "LAND"
    state = IDLE

    levels: List[Level] = []
    ratio = args.ratio

    print("[keys] SPACE: start/stop TAKEOFF mapping (HOME set to center on first press)")
    print("[keys] L: LAND auto (auto level selection)  |  M: stop landing (back to IDLE)")
    print("[keys] R: reset  |  [ / ]: prev/next camera  |  Q/ESC: quit")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("[warn] frame grab failed")
            time.sleep(0.05)
            continue

        # Prepare gray + display
        if frame.ndim == 3 and frame.shape[2] == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            disp = frame.copy()
        elif frame.ndim == 2:
            if frame.dtype == np.uint16:
                norm = cv2.convertScaleAbs(frame, alpha=255.0/65535.0)
                gray = norm
                disp = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)
            else:
                gray = frame
                disp = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            disp = frame.copy()

        overlay = disp
        h, w = gray.shape[:2]
        cx, cy = w*0.5, h*0.5

        # HUD info
        info = [
            f"STATE: {state}",
            f"Device: {device}  Backend: {args.backend}  FOURCC: {args.fourcc or 'default'}  raw={args.raw}",
            f"Levels: {len(levels)}  |  FAST={args.fast_thresh}  kptsMax={args.max_kpts}",
            f"FOV: {args.fov} deg  |  {w}x{h}",
        ]

        # Draw HOME of the best current reference, if any
        current_home = None

        # ---- TAKEOFF: auto-capture levels by scale changes ----
        if state == TAKEOFF:
            # First level?
            if len(levels) == 0:
                kps = detect_fast(gray, args.fast_thresh, args.max_kpts, grid=(args.grid_x, args.grid_y))
                kps, desc = compute_orb(gray, kps)
                home_pt = (cx, cy)  # center at first SPACE
                alt_m = alt_list[0] if alt_list and len(alt_list) > 0 else None
                levels.append(Level(gray.copy(), kps, desc, alt_m, home_pt, scale_from_lvl0=1.0))
                print(f"[takeoff] Level 0 captured. HOME={home_pt}, alt={alt_m}")
                current_home = home_pt
            else:
                # Compare live to last level
                Lprev = levels[-1]
                kps_live = detect_fast(gray, args.fast_thresh, args.max_kpts, grid=(args.grid_x, args.grid_y))
                kps_live, desc_live = compute_orb(gray, kps_live)
                matches = match_knn(desc_live, Lprev.desc, ratio=ratio)
                H, inliers = estimate_h(kps_live, Lprev.kps, matches, reproj_thresh=args.reproj)
                inl = sum(inliers) if inliers is not None else 0
                info.append(f"TAKEOFF match: {len(matches)}  inliers: {inl}")
                if H is not None and inl >= 20:
                    H = H / H[2,2]
                    # collect matched points for robust scale estimate
                    pts_prev = np.float32([Lprev.kps[m.trainIdx].pt for m in matches])
                    pts_live = np.float32([kps_live[m.queryIdx].pt for m in matches])
                    s_rel = decompose_scale(H, pts_prev, pts_live)  # current vs previous
                    s_total = Lprev.scale_from_lvl0 * s_rel

                    # Propagate HOME into live
                    prev_home = np.array([[[Lprev.home_pt[0], Lprev.home_pt[1]]]], dtype=np.float32)
                    proj = cv2.perspectiveTransform(prev_home, H)  # prev->live
                    home_pt = (float(proj[0,0,0]), float(proj[0,0,1]))
                    current_home = home_pt

                    # Show debug on HUD
                    info.append(f"s_rel={s_rel:.3f}  s_total={s_total:.3f}")

                    # Adaptive spacing (configurable)
                    n = len(levels)
                    if n < 3:
                        step = args.step_near
                    elif n < 6:
                        step = args.step_mid
                    else:
                        step = args.step_far

                    # Fallback: if points have shifted a lot, consider capturing even if scale didn't cross threshold
                    med_shift = 0.0
                    if len(pts_prev) >= 8:
                        shifts = np.linalg.norm(pts_live - pts_prev, axis=1)
                        med_shift = float(np.median(shifts))

                    should_capture = (s_rel < step) or (med_shift > args.flow_px and inl >= 20)

                    if should_capture:
                        alt_m = alt_list[n] if alt_list and n < len(alt_list) else None
                        levels.append(Level(gray.copy(), kps_live, desc_live, alt_m, home_pt, s_total))
                        print(f"[takeoff] Level {n} captured. s_rel={s_rel:.3f}, s_total={s_total:.3f}, medShift={med_shift:.1f}px, alt={alt_m}, HOME={home_pt}")
                else:
                    info.append("TAKEOFF: tracking weak (no H or low inliers)")

        # ---- LAND: automatically choose best reference level & compute HOME offset ----
        best_idx = None
        best_inliers = -1
        best_H = None
        if state == LAND and len(levels) > 0:
            # Try match against all levels (or a small neighborhood around last best).
            kps_live = detect_fast(gray, args.fast_thresh, args.max_kpts, grid=(args.grid_x, args.grid_y))
            kps_live, desc_live = compute_orb(gray, kps_live)

            for i, L in enumerate(levels):
                matches = match_knn(desc_live, L.desc, ratio=ratio)
                H, inliers = estimate_h(kps_live, L.kps, matches, reproj_thresh=args.reproj)
                inl = sum(inliers) if inliers is not None else 0
                if inl > best_inliers:
                    best_inliers = inl
                    best_idx = i
                    best_H = H

            if best_idx is not None and best_H is not None and best_inliers >= 20:
                L = levels[best_idx]
                info.append(f"LAND: best level={best_idx}  inliers={best_inliers}")
                H = best_H / best_H[2,2]

                # Project HOME from ref level into live
                ref_home = np.array([[[L.home_pt[0], L.home_pt[1]]]], dtype=np.float32)
                H_inv = np.linalg.inv(H)  # ref->live
                proj = cv2.perspectiveTransform(ref_home, H_inv)
                hx, hy = float(proj[0,0,0]), float(proj[0,0,1])
                current_home = (hx, hy)

                # offset from frame center
                dx_px = hx - cx
                dy_px = hy - cy
                dx_m = px_to_m(dx_px, w, args.fov, L.alt_m) if L.alt_m is not None else None
                dy_m = px_to_m(dy_px, w, args.fov, L.alt_m) if L.alt_m is not None else None

                # draw
                cv2.drawMarker(overlay, (int(hx), int(hy)), (0,255,255), cv2.MARKER_CROSS, 26, 2)
                cv2.arrowedLine(overlay, (int(cx), int(cy)), (int(hx), int(hy)), (0,255,0), 2, tipLength=0.15)
                info.append(f"Offset to HOME: {dx_px:+.1f}px, {dy_px:+.1f}px" + (f" | {dx_m:+.3f} m, {dy_m:+.3f} m" if dx_m is not None else ""))

            else:
                info.append("LAND: no good lock (move slowly / add texture)")

        # Draw HOME marker (if known)
        if current_home is not None:
            cv2.circle(overlay, (int(current_home[0]), int(current_home[1])), 6, (0,255,255), 2)

        # HUD
        y0 = 24
        for s in info:
            cv2.putText(overlay, s, (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
            cv2.putText(overlay, s, (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
            y0 += 22

        cv2.imshow("Webcam RTH demo (auto levels)", overlay)
        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord('q'), ord('Q')):
            break
        elif key in (ord('r'), ord('R')):
            levels.clear()
            state = IDLE
            print("[info] Reset.")
        elif key in (ord('m'), ord('M')):
            # stop landing / return idle
            if state == LAND:
                state = IDLE
                print("[info] LAND canceled -> IDLE")
        elif key == ord(' '):  # SPACE
            if state == IDLE:
                state = TAKEOFF
                print("[info] TAKEOFF started. HOME will be set to frame center on first capture.")
            elif state == TAKEOFF:
                # toggle pause/resume mapping
                state = IDLE
                print("[info] TAKEOFF paused -> IDLE (press SPACE again to resume)")
            else:
                # ignore in LAND
                pass
        elif key in (ord('l'), ord('L')):
            state = LAND
            print("[info] LAND auto mode — choosing best level each frame.")
        elif key in (ord('f'), ord('F')) and state == TAKEOFF and len(levels) > 0:
            # Force a new level using current view and propagated HOME from previous (if possible)
            Lprev = levels[-1]
            # try to propagate HOME; if fail, use center
            kps_live = detect_fast(gray, args.fast_thresh, args.max_kpts, grid=(args.grid_x, args.grid_y))
            kps_live, desc_live = compute_orb(gray, kps_live)
            matches = match_knn(desc_live, Lprev.desc, ratio=ratio)
            H, inliers = estimate_h(kps_live, Lprev.kps, matches, reproj_thresh=args.reproj)
            if H is not None and inliers is not None and sum(inliers) >= 10:
                H = H / H[2,2]
                prev_home = np.array([[[Lprev.home_pt[0], Lprev.home_pt[1]]]], dtype=np.float32)
                proj = cv2.perspectiveTransform(prev_home, H)
                home_pt = (float(proj[0,0,0]), float(proj[0,0,1]))
            else:
                home_pt = (cx, cy)
            alt_m = None
            n = len(levels)
            if alt_list and n < len(alt_list):
                alt_m = alt_list[n]
            levels.append(Level(gray.copy(), kps_live, desc_live, alt_m, home_pt, Lprev.scale_from_lvl0))
            print(f"[takeoff] FORCED capture: Level {n}  HOME={home_pt}  alt={alt_m}")
        elif key == ord(']'):
            new_dev = next_device(device, +1, args.max_devices)
            if new_dev != device:
                cap.release()
                cap = try_open(new_dev, BACKENDS.get(args.backend.lower(), 0), width, height, args.fourcc, not args.raw)
                if cap:
                    device = new_dev
                    print(f"[info] Switched to device {device}.")
                else:
                    print(f"[warn] Could not open device {new_dev}.")
                    cap = try_open(device, BACKENDS.get(args.backend.lower(), 0), width, height, args.fourcc, not args.raw)
        elif key == ord('['):
            new_dev = next_device(device, -1, args.max_devices)
            if new_dev != device:
                cap.release()
                cap = try_open(new_dev, BACKENDS.get(args.backend.lower(), 0), width, height, args.fourcc, not args.raw)
                if cap:
                    device = new_dev
                    print(f"[info] Switched to device {device}.")
                else:
                    print(f"[warn] Could not open device {new_dev}.")
                    cap = try_open(device, BACKENDS.get(args.backend.lower(), 0), width, height, args.fourcc, not args.raw)

    cap.release()
    cv2.destroyAllWindows()

def parse_args():
    ap = argparse.ArgumentParser(description="Webcam-based RTH demo: auto map levels on takeoff, auto select on landing.")

    ap.add_argument("--device", type=int, default=0, help="Camera device index.")
    ap.add_argument("--max-devices", type=int, default=10, help="Max indices to scan/cycle.")
    ap.add_argument("--backend", type=str, default="auto", choices=list(BACKENDS.keys()), help="Capture backend (macOS=avfoundation, Windows=msmf/dshow, Linux=v4l2).")
    ap.add_argument("--list", action="store_true", help="List available camera indices and exit.")
    ap.add_argument("--resolution", type=str, default="640x480", help="Capture resolution WxH, e.g., 640x480.")
    ap.add_argument("--fourcc", type=str, default="", help="FOURCC code to request (e.g., MJPG, YUY2, Y16 ).")
    ap.add_argument("--raw", action="store_true", help="Disable OpenCV RGB conversion (needed for raw Y16/IR streams).")
    ap.add_argument("--fast-thresh", type=int, default=16, help="FAST corner threshold.")
    ap.add_argument("--max-kpts", type=int, default=500, help="Max keypoints per frame.")
    ap.add_argument("--grid-x", type=int, default=20, help="Grid columns for keypoint spread.")
    ap.add_argument("--grid-y", type=int, default=15, help="Grid rows for keypoint spread.")
    ap.add_argument("--ratio", type=float, default=0.75, help="Lowe ratio for matching.")
    ap.add_argument("--reproj", type=float, default=2.0, help="RANSAC reprojection thresh (px).")
    ap.add_argument("--fov", type=float, default=60.0, help="Horizontal FOV in degrees (for px->meters).")
    ap.add_argument("--alts", type=str, default="", help="Comma list of altitudes (meters) for each level; optional.")
    ap.add_argument("--step-near", type=float, default=0.98, help="Relative scale threshold vs previous to add a level for first ~3 levels (default 0.98).")
    ap.add_argument("--step-mid", type=float, default=0.95, help="Relative scale threshold for mid altitudes (default 0.95).")
    ap.add_argument("--step-far", type=float, default=0.90, help="Relative scale threshold for higher altitudes (default 0.90).")
    ap.add_argument("--flow-px", type=float, default=12.0, help="Fallback: if median keypoint shift exceeds this many pixels and inliers>=20, add a level anyway.")
    return ap.parse_args()

if __name__ == "__main__":
    run(parse_args())
