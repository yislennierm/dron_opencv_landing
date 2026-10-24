#!/usr/bin/env python3
"""
FAST + ORB profiler demo
------------------------
This script helps you pinpoint which stage is slow when working with large top‑down images.

It instruments each step:
  - image load + downscale/crop
  - FAST detection (+ grid spread)
  - ORB/BRIEF descriptor compute
  - matching (Hamming + ratio test)
  - homography (RANSAC)
  - visualization and save

Usage examples:
  pip install opencv-python numpy
  python fast_orb_profiler.py --image path/to/your_big_image.jpg --downscale 0.5 --iterations 5
  python fast_orb_profiler.py --image path.jpg --target-mp 0.6
  python fast_orb_profiler.py --image path.jpg --use-orb-detector  # compare ORB's built-in detector vs FAST

Notes:
- If you have opencv-contrib installed, you can try --descriptor brief to compare speed vs ORB's descriptor.
- Use --reuse-kp / --reuse-desc to isolate descriptor or matching costs.
"""

import argparse
import os
import sys
import time
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np
import cv2

# ---------------------------- Timing helpers ---------------------------------

@dataclass
class StageStat:
    name: str
    ms: float

class Timer:
    def __init__(self, name: str, sink: List[StageStat]):
        self.name = name
        self.sink = sink
        self.t0 = None
    def __enter__(self):
        self.t0 = time.perf_counter()
        return self
    def __exit__(self, exc_type, exc, tb):
        t1 = time.perf_counter()
        self.sink.append(StageStat(self.name, (t1 - self.t0) * 1000.0))

def fmt_ms(ms: float) -> str:
    return f"{ms:7.2f} ms"

def print_stats(stats: List[StageStat], header: str = "Stage timings"):
    total = sum(s.ms for s in stats)
    print(f"\n== {header} ==")
    for s in stats:
        pct = (s.ms / total * 100.0) if total > 1e-9 else 0.0
        print(f"{s.name:22s}: {fmt_ms(s.ms)}  ({pct:5.1f}%)")
    print(f"{'-'*22}: {fmt_ms(total)}  (100.0%)\n")

# ---------------------------- Image helpers ----------------------------------

def auto_downscale_factor(w: int, h: int, target_megapixels: float) -> float:
    mp = (w * h) / 1_000_000.0
    if mp <= target_megapixels or target_megapixels <= 0:
        return 1.0
    scale = (target_megapixels / mp) ** 0.5
    return float(scale)

def center_crop(img: np.ndarray, frac: float) -> np.ndarray:
    if frac >= 0.999:
        return img
    h, w = img.shape[:2]
    cw = int(w * frac)
    ch = int(h * frac)
    x0 = (w - cw) // 2
    y0 = (h - ch) // 2
    return img[y0:y0+ch, x0:x0+cw]

# ---------------------------- Keypoint utils ---------------------------------

def grid_keep_top_k(keypoints: List[cv2.KeyPoint], max_total: int, grid: Tuple[int, int], image_shape) -> List[cv2.KeyPoint]:
    """Uniform spatial spread: divide image into grid cells and keep top few by response per cell, then cap globally."""
    h, w = image_shape[:2]
    gx, gy = grid
    cell_w = w / gx
    cell_h = h / gy
    # buckets[y][x] -> list of kps
    buckets: List[List[List[cv2.KeyPoint]]] = [[[] for _ in range(gx)] for _ in range(gy)]
    for kp in keypoints:
        cx = min(gx - 1, max(0, int(kp.pt[0] / cell_w)))
        cy = min(gy - 1, max(0, int(kp.pt[1] / cell_h)))
        buckets[cy][cx].append(kp)
    selected: List[cv2.KeyPoint] = []
    # take up to N per cell to ensure spread
    PER_CELL = 3
    for y in range(gy):
        for x in range(gx):
            cell = buckets[y][x]
            if not cell:
                continue
            cell.sort(key=lambda k: k.response, reverse=True)
            selected.extend(cell[:PER_CELL])
    # global trim
    selected.sort(key=lambda k: k.response, reverse=True)
    return selected[:max_total]

# ---------------------------- Pipeline steps ---------------------------------

def detect_keypoints(gray: np.ndarray, args) -> List[cv2.KeyPoint]:
    if args.use_orb_detector:
        orb = cv2.ORB_create(nfeatures=args.max_kpts, edgeThreshold=19, patchSize=31, fastThreshold=max(1, args.fast_thresh))
        kps = orb.detect(gray, None)
    else:
        fast = cv2.FastFeatureDetector_create(threshold=args.fast_thresh,
                                              nonmaxSuppression=True,
                                              type=cv2.FAST_FEATURE_DETECTOR_TYPE_9_16)
        kps = fast.detect(gray, None)
    # spread & cap
    kps = grid_keep_top_k(kps, max_total=args.max_kpts, grid=(args.grid_x, args.grid_y), image_shape=gray.shape)
    return kps

def compute_descriptors(gray: np.ndarray, kps: List[cv2.KeyPoint], args) -> Tuple[List[cv2.KeyPoint], Optional[np.ndarray], str]:
    # choose descriptor
    desc_name = "ORB(256)"
    if args.descriptor.lower() == "brief":
        # Try to use BRIEF from xfeatures2d if available
        try:
            brief = cv2.xfeatures2d.BriefDescriptorExtractor_create(bytes=32)
            # need orientation for rotation invariance? Use ORB's orientation computation only
            # We'll rely on BRIEF as-is (faster but less robust to rotation).
            kps, desc = brief.compute(gray, kps)
            return kps, desc, "BRIEF(256)"
        except Exception:
            print("[info] BRIEF not available (opencv-contrib not installed). Falling back to ORB.")
            # fall through to ORB
    orb = cv2.ORB_create(nfeatures=max(args.max_kpts, 2000), edgeThreshold=19, patchSize=31, fastThreshold=0)
    kps, desc = orb.compute(gray, kps)
    return kps, desc, "ORB(256)"

def match_descriptors(desc1: np.ndarray, desc2: np.ndarray, args) -> List[cv2.DMatch]:
    if desc1 is None or desc2 is None or len(desc1) == 0 or len(desc2) == 0:
        return []
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    raw = bf.knnMatch(desc1, desc2, k=2)
    good = []
    for pair in raw:
        if len(pair) < 2:
            continue
        m, n = pair
        if m.distance < args.ratio * n.distance:
            good.append(m)
    return good

def estimate_homography(kps1: List[cv2.KeyPoint], kps2: List[cv2.KeyPoint], matches: List[cv2.DMatch], args):
    if len(matches) < 4:
        return None, None
    pts1 = np.float32([kps1[m.queryIdx].pt for m in matches])
    pts2 = np.float32([kps2[m.trainIdx].pt for m in matches])
    H, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, args.reproj)
    inliers = mask.ravel().tolist() if mask is not None else None
    return H, inliers

def draw_results(img1c, img2c, kps1, kps2, matches, inliers, H):
    if inliers is not None:
        inlier_matches = [m for m, keep in zip(matches, inliers) if keep]
    else:
        inlier_matches = matches
    vis = cv2.drawMatches(img1c, kps1, img2c, kps2, inlier_matches, None,
                          matchColor=(0,255,0), singlePointColor=(255,0,0),
                          flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    if H is not None:
        h, w = img1c.shape[:2]
        box = np.float32([[0,0],[w,0],[w,h],[0,h]]).reshape(-1,1,2)
        proj = cv2.perspectiveTransform(box, H)
        proj_int = np.int32(proj + np.array([[[w,0]]], dtype=np.int32))  # shift due to concatenation
        cv2.polylines(vis, [proj_int], True, (255,0,255), 3, cv2.LINE_AA)
    return vis

# ---------------------------- Main -------------------------------------------

def synthetic_pair(w=1280, h=960):
    # Generate a textured checkerboard + shapes, and a warped version
    img = np.zeros((h, w), np.uint8)
    squares = 12
    sw = w // squares
    sh = h // squares
    for y in range(squares):
        for x in range(squares):
            img[y*sh:(y+1)*sh, x*sw:(x+1)*sw] = 220 if (x+y)%2==0 else 30
    cv2.circle(img, (int(0.75*w), int(0.25*h)), 60, 150, -1)
    cv2.circle(img, (int(0.25*w), int(0.75*h)), 40, 90, -1)

    # apply transform
    angle, scale = 10, 0.95
    M = cv2.getRotationMatrix2D((w/2, h/2), angle, scale)
    rotated = cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_LINEAR)

    src = np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]])
    dst = np.float32([[40,30],[w-70,10],[w-30,h-50],[25,h-20]])
    H = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(rotated, H, (w, h), flags=cv2.INTER_LINEAR)
    return img, warped

def main():
    ap = argparse.ArgumentParser(description="Profile FAST+ORB pipeline step-by-step.")
    ap.add_argument("--image", type=str, default="", help="Path to a top-down image. If set, we create a synthetic second view.")
    ap.add_argument("--second", type=str, default="", help="Optional second image. If omitted, a synthetic transform of --image is used.")
    ap.add_argument("--downscale", type=float, default=1.0, help="Uniform downscale factor applied after load (e.g., 0.5).")
    ap.add_argument("--target-mp", type=float, default=0.0, help="Auto downscale to this many megapixels (e.g., 0.6). Overrides --downscale if >0.")
    ap.add_argument("--crop-frac", type=float, default=1.0, help="Center crop fraction (0<frac<=1). Use 0.6..0.9 to speed up.")
    ap.add_argument("--max-kpts", type=int, default=600, help="Max keypoints per image after grid spread.")
    ap.add_argument("--grid-x", type=int, default=20, help="Grid columns for keypoint spread.")
    ap.add_argument("--grid-y", type=int, default=15, help="Grid rows for keypoint spread.")
    ap.add_argument("--fast-thresh", type=int, default=16, help="FAST threshold (if not using ORB detector).")
    ap.add_argument("--use-orb-detector", action="store_true", help="Use ORB detector instead of FAST (compare speed/quality).")
    ap.add_argument("--descriptor", type=str, default="orb", choices=["orb","brief"], help="Descriptor type.")
    ap.add_argument("--ratio", type=float, default=0.75, help="Lowe ratio for KNN match.")
    ap.add_argument("--reproj", type=float, default=2.0, help="RANSAC reprojection threshold (px).")
    ap.add_argument("--iterations", type=int, default=1, help="Repeat core stages to average timings.")
    ap.add_argument("--reuse-kp", action="store_true", help="Reuse keypoints across iterations (isolates descriptor/matching cost).")
    ap.add_argument("--reuse-desc", action="store_true", help="Reuse descriptors across iterations (isolates matching/homography cost).")
    ap.add_argument("--no-h", action="store_true", help="Skip homography estimation (focus on detect/describe/match).")
    ap.add_argument("--save", type=str, default="fast_orb_profiler_output.png", help="Save visualization here.")
    ap.add_argument("--no-viz", action="store_true", help="Skip drawing/saving visualization.")
    ap.add_argument("--show", action="store_true", help="Show window at the end (may block).")
    args = ap.parse_args()

    stats: List[StageStat] = []

    # Load or synthesize images
    if args.image and os.path.exists(args.image):
        with Timer("load_image1", stats):
            img1 = cv2.imread(args.image, cv2.IMREAD_GRAYSCALE)
        if img1 is None:
            print("[error] Failed to read --image.")
            sys.exit(1)
        if args.second:
            with Timer("load_image2", stats):
                img2 = cv2.imread(args.second, cv2.IMREAD_GRAYSCALE)
            if img2 is None:
                print("[error] Failed to read --second.")
                sys.exit(1)
        else:
            with Timer("synthesize_img2", stats):
                # make a second view by transforming img1
                h, w = img1.shape[:2]
                angle, scale = 8, 0.98
                M = cv2.getRotationMatrix2D((w/2, h/2), angle, scale)
                tmp = cv2.warpAffine(img1, M, (w, h), flags=cv2.INTER_LINEAR)
                src = np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]])
                dst = np.float32([[35,20],[w-60,15],[w-40,h-40],[20,h-15]])
                H = cv2.getPerspectiveTransform(src, dst)
                img2 = cv2.warpPerspective(tmp, H, (w, h), flags=cv2.INTER_LINEAR)
    else:
        with Timer("synthesize_pair", stats):
            img1, img2 = synthetic_pair(1600, 1200)

    # Optional crop
    if args.crop_frac < 0.999:
        with Timer("center_crop", stats):
            img1 = center_crop(img1, args.crop_frac)
            img2 = center_crop(img2, args.crop_frac)

    # Downscale handling
    h0, w0 = img1.shape[:2]
    if args.target_mp and args.target_mp > 0.0:
        scale = auto_downscale_factor(w0, h0, args.target_mp)
    else:
        scale = args.downscale
    if scale != 1.0:
        with Timer(f"downscale x{scale:.3f}", stats):
            nw = max(32, int(w0 * scale))
            nh = max(32, int(h0 * scale))
            img1 = cv2.resize(img1, (nw, nh), interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR)
            img2 = cv2.resize(img2, (nw, nh), interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR)

    h, w = img1.shape[:2]
    mp = (w*h)/1_000_000.0
    print(f"[info] Image size after preprocess: {w}x{h} ({mp:.2f} MP)")

    # Prepare color copies for visualization later
    img1c = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
    img2c = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)

    # Core pipeline — optionally iterate to average timings
    # Optionally reuse kps/desc across iterations to isolate certain costs
    kps1_cache = desc1_cache = None
    kps2_cache = desc2_cache = None

    totals: List[StageStat] = []

    for it in range(args.iterations):
        iter_stats: List[StageStat] = []
        print(f"\n-- Iteration {it+1}/{args.iterations} --")

        # Detect
        if args.reuse_kp and kps1_cache is not None and kps2_cache is not None:
            kps1 = kps1_cache
            kps2 = kps2_cache
        else:
            with Timer("detect_kp_img1", iter_stats):
                kps1 = detect_keypoints(img1, args)
            with Timer("detect_kp_img2", iter_stats):
                kps2 = detect_keypoints(img2, args)
            if args.reuse_kp:
                kps1_cache, kps2_cache = kps1, kps2

        print(f"   keypoints: img1={len(kps1)}, img2={len(kps2)}")

        # Describe
        if args.reuse_desc and desc1_cache is not None and desc2_cache is not None:
            desc1 = desc1_cache
            desc2 = desc2_cache
            desc_name = "cached"
        else:
            with Timer("describe_img1", iter_stats):
                kps1, desc1, desc_name = compute_descriptors(img1, kps1, args)
            with Timer("describe_img2", iter_stats):
                kps2, desc2, _ = compute_descriptors(img2, kps2, args)
            if args.reuse_desc:
                desc1_cache, desc2_cache = desc1, desc2

        if desc1 is None or desc2 is None or len(desc1)==0 or len(desc2)==0:
            print("[warn] No descriptors — try lowering --fast-thresh or downscaling less.")
            break

        print(f"   descriptor: {desc_name}, desc1={desc1.shape}, desc2={desc2.shape}")

        # Match
        with Timer("match_knn+ratio", iter_stats):
            matches = match_descriptors(desc1, desc2, args)
        print(f"   matches (ratio {args.ratio:.2f}): {len(matches)}")

        # Homography
        if args.no_h:
            H, inliers = None, None
        else:
            with Timer("homography_RANSAC", iter_stats):
                H, inliers = estimate_homography(kps1, kps2, matches, args)
        in_count = sum(inliers) if inliers is not None else 0
        print(f"   inliers: {in_count}{' (H found)' if H is not None else ''}")

        # Record iteration stats
        print_stats(iter_stats, header=f"Iteration {it+1} timings")
        totals.extend(iter_stats)

    # Aggregate stats across iterations
    if args.iterations > 1 and totals:
        # Sum per-name
        agg = {}
        for s in totals:
            agg.setdefault(s.name, 0.0)
            agg[s.name] += s.ms
        agg_stats = [StageStat(k, v/args.iterations) for k, v in sorted(agg.items())]
        print_stats(agg_stats, header=f"Average per-iteration timings over {args.iterations} iters")

    # Visualization/save (only for last iteration's outputs)
    if not args.no_viz:
        with Timer("visualize+save", stats):
            vis = draw_results(img1c, img2c, kps1 if 'kps1' in locals() else [],
                               kps2 if 'kps2' in locals() else [],
                               matches if 'matches' in locals() else [],
                               inliers if 'inliers' in locals() else None,
                               H if 'H' in locals() else None)
            cv2.imwrite(args.save, vis)
        print(f"[info] Saved visualization to: {args.save}")

    # Print top-level stats (load, synthesize, crop, downscale, visualize)
    print_stats(stats, header="Top-level (I/O & preprocess & viz)")

    # Exit code hinting
    if 'matches' in locals() and len(matches) >= 4 and (H is not None or args.no_h):
        sys.exit(0)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()
