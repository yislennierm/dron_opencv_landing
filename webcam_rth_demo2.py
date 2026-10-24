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

class Level:
    def __init__(self, img_gray, kps, desc, alt_m: Optional[float] = None):
        self.img_gray = img_gray
        self.kps = kps
        self.desc = desc
        self.alt_m = alt_m

def run(args):
    cap, backend = open_camera_or_exit(args)
    device = args.device
    width, height = parse_resolution(args.resolution)
    alt_list = [float(x) for x in args.alts.split(',') if x.strip()] if args.alts else None

    mode = "MAP"
    levels: List[Level] = []
    land_level_idx = -1
    ratio = args.ratio

    print("[keys] SPACE: (MAP) capture level / (LAND) step to next lower level")
    print("[keys] L: start LAND  |  M: back to MAP  |  R: reset levels  |  [ / ]: prev/next camera  |  Q/ESC: quit")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("[warn] frame grab failed")
            time.sleep(0.05)
            continue

        # Handle color/mono/IR frames
        if frame.ndim == 3 and frame.shape[2] == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            disp = frame.copy()
        elif frame.ndim == 2:
            # mono (8-bit) or 16-bit
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

        info_lines = [
            f"MODE: {mode}",
            f"Device: {device}  Backend: {args.backend}  FOURCC: {args.fourcc or 'default'}  raw={args.raw}",
            f"Levels: {len(levels)}",
            f"FAST thresh: {args.fast_thresh}, max_kpts: {args.max_kpts}",
            f"FOV: {args.fov} deg  |  Resolution: {w}x{h}",
        ]

        if mode == "LAND" and 0 <= land_level_idx < len(levels):
            kps_live = detect_fast(gray, args.fast_thresh, args.max_kpts, grid=(args.grid_x, args.grid_y))
            kps_live, desc_live = compute_orb(gray, kps_live)

            L = levels[land_level_idx]
            matches = match_knn(desc_live, L.desc, ratio=ratio)
            H, inliers = estimate_h(kps_live, L.kps, matches, reproj_thresh=args.reproj)

            inliers_count = sum(inliers) if inliers is not None else 0
            info_lines.append(f"Matches: {len(matches)}  Inliers: {inliers_count}")

            dx_px = dy_px = None
            dx_m = dy_m = None
            if H is not None:
                H = H / H[2,2]
                cx, cy = w*0.5, h*0.5
                pt = np.array([[[cx, cy]]], dtype=np.float32)
                H_inv = np.linalg.inv(H)
                proj = cv2.perspectiveTransform(pt, H_inv)
                px, py = float(proj[0,0,0]), float(proj[0,0,1])
                dx_px = (px - cx)
                dy_px = (py - cy)

                p0 = (int(cx), int(cy))
                p1 = (int(cx + dx_px), int(cy + dy_px))
                cv2.arrowedLine(overlay, p0, p1, (0,255,0), 2, tipLength=0.15)
                cv2.drawMarker(overlay, p0, (255,0,255), cv2.MARKER_CROSS, 20, 2)

                dx_m = px_to_m(dx_px, w, args.fov, L.alt_m)
                dy_m = px_to_m(dy_px, w, args.fov, L.alt_m)

            if dx_px is not None:
                info_lines.append(f"Offset: {dx_px:+.1f}px, {dy_px:+.1f}px")
            if dx_m is not None:
                info_lines.append(f"Offset: {dx_m:+.3f} m, {dy_m:+.3f} m (alt {L.alt_m} m)")

            info_lines.append(f"LAND level idx: {land_level_idx} / {len(levels)-1}")

        # HUD
        y0 = 24
        for s in info_lines:
            cv2.putText(overlay, s, (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
            cv2.putText(overlay, s, (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
            y0 += 22

        cv2.imshow("Webcam RTH demo (FAST+ORB)", overlay)
        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord('q'), ord('Q')):
            break
        elif key in (ord('r'), ord('R')):
            levels.clear()
            land_level_idx = -1
            mode = "MAP"
            print("[info] Levels reset.")
        elif key in (ord('m'), ord('M')):
            mode = "MAP"
            print("[info] Switched to MAP mode.")
        elif key in (ord('l'), ord('L')):
            if len(levels) == 0:
                print("[warn] No levels captured yet.")
            else:
                mode = "LAND"
                land_level_idx = len(levels) - 1
                print(f"[info] LAND mode. Starting at level index {land_level_idx}. SPACE steps lower.")
        elif key == 32:  # SPACE
            if mode == "MAP":
                kps = detect_fast(gray, args.fast_thresh, args.max_kpts, grid=(args.grid_x, args.grid_y))
                kps, desc = compute_orb(gray, kps)
                alt_m = None
                if alt_list is not None:
                    idx = len(levels)
                    alt_m = alt_list[idx] if idx < len(alt_list) else None
                levels.append(Level(gray.copy(), kps, desc, alt_m))
                print(f"[info] Captured level {len(levels)-1}  (kps={len(kps)})  alt={alt_m}")
            elif mode == "LAND":
                if land_level_idx > 0:
                    land_level_idx -= 1
                    print(f"[info] LAND: stepped to lower level idx {land_level_idx}.")
                else:
                    print("[info] LAND: already at lowest level.")
        elif key == ord(']'):
            new_dev = next_device(device, +1, args.max_devices)
            if new_dev != device:
                cap.release()
                cap = try_open(new_dev, backend, width, height, args.fourcc, not args.raw)
                if cap:
                    device = new_dev
                    print(f"[info] Switched to device {device}.")
                else:
                    print(f"[warn] Could not open device {new_dev}.")
                    cap = try_open(device, backend, width, height, args.fourcc, not args.raw)
        elif key == ord('['):
            new_dev = next_device(device, -1, args.max_devices)
            if new_dev != device:
                cap.release()
                cap = try_open(new_dev, backend, width, height, args.fourcc, not args.raw)
                if cap:
                    device = new_dev
                    print(f"[info] Switched to device {device}.")
                else:
                    print(f"[warn] Could not open device {new_dev}.")
                    cap = try_open(device, backend, width, height, args.fourcc, not args.raw)

    cap.release()
    cv2.destroyAllWindows()

def parse_args():
    ap = argparse.ArgumentParser(description="Interactive webcam-based RTH learning demo (FAST+ORB) with camera selection & IR support.")
    ap.add_argument("--device", type=int, default=0, help="Camera device index.")
    ap.add_argument("--max-devices", type=int, default=10, help="Max indices to scan/cycle.")
    ap.add_argument("--backend", type=str, default="auto", choices=list(BACKENDS.keys()), help="Capture backend (macOS=avfoundation, Windows=msmf/dshow, Linux=v4l2).")
    ap.add_argument("--list", action="store_true", help="List available camera indices and exit.")
    ap.add_argument("--resolution", type=str, default="640x480", help="Capture resolution WxH, e.g., 640x480.")
    ap.add_argument("--fourcc", type=str, default="", help="FOURCC code to request (e.g., MJPG, YUY2, Y16 ). Use 'Y16 ' (with space) for 16‑bit IR on some cameras.")
    ap.add_argument("--raw", action="store_true", help="Disable OpenCV RGB conversion (needed for raw Y16/IR streams).")
    ap.add_argument("--fast-thresh", type=int, default=16, help="FAST corner threshold.")
    ap.add_argument("--max-kpts", type=int, default=500, help="Max keypoints per frame.")
    ap.add_argument("--grid-x", type=int, default=20, help="Grid columns for keypoint spread.")
    ap.add_argument("--grid-y", type=int, default=15, help="Grid rows for keypoint spread.")
    ap.add_argument("--ratio", type=float, default=0.75, help="Lowe ratio for matching.")
    ap.add_argument("--reproj", type=float, default=2.0, help="RANSAC reprojection thresh (px).")
    ap.add_argument("--fov", type=float, default=60.0, help="Horizontal FOV in degrees (for px->meters).")
    ap.add_argument("--alts", type=str, default="", help="Comma list of altitudes (meters) for each captured level, e.g., '0.3,0.6,1.0'")
    return ap.parse_args()

if __name__ == "__main__":
    run(parse_args())
