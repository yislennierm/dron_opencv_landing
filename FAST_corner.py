import argparse
import math
import os
import sys
import cv2
import numpy as np

def make_checkerboard(w=800, h=600, squares=10):
    img = np.zeros((h, w), np.uint8)
    sw = w // squares
    sh = h // squares
    for y in range(squares):
        for x in range(squares):
            if (x + y) % 2 == 0:
                img[y*sh:(y+1)*sh, x*sw:(x+1)*sw] = 220
            else:
                img[y*sh:(y+1)*sh, x*sw:(x+1)*sw] = 30
    # add a few circles to break symmetry
    cv2.circle(img, (int(0.75*w), int(0.25*h)), 40, 150, -1)
    cv2.circle(img, (int(0.25*w), int(0.75*h)), 30, 90, -1)
    return img

def synthetic_transform(img):
    h, w = img.shape[:2]
    # rotation + scale + perspective + translation
    angle = 12  # deg
    scale = 0.9
    M = cv2.getRotationMatrix2D((w/2, h/2), angle, scale)
    rotated = cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_LINEAR)

    src = np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]])
    dst = np.float32([[40,30],[w-60,10],[w-20,h-40],[20,h-10]])
    H = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(rotated, H, (w, h), flags=cv2.INTER_LINEAR)
    return warped

def grid_keep_top_k(keypoints, max_total=500, grid=(20, 15), image_shape=None):
    """
    Keep a uniform spatial spread: divide image into grid cells and keep
    the best (highest response) few overall until we reach max_total.
    """
    if image_shape is None:
        raise ValueError("image_shape required")
    h, w = image_shape[:2]
    gx, gy = grid
    cell_w = w / gx
    cell_h = h / gy
    # bucket keypoints
    buckets = [[[] for _ in range(gx)] for _ in range(gy)]
    for kp in keypoints:
        cx = min(gx-1, max(0, int(kp.pt[0] / cell_w)))
        cy = min(gy-1, max(0, int(kp.pt[1] / cell_h)))
        buckets[cy][cx].append(kp)
    # within each cell, sort by response (desc) and take top 3 (tweakable)
    selected = []
    for y in range(gy):
        for x in range(gx):
            cell = buckets[y][x]
            cell.sort(key=lambda k: k.response, reverse=True)
            selected.extend(cell[:3])
    # if too many, trim globally by response
    selected.sort(key=lambda k: k.response, reverse=True)
    return selected[:max_total]

def detect_fast(gray, threshold=16, nonmax=True, max_kpts=500):
    fast = cv2.FastFeatureDetector_create(threshold=threshold, nonmaxSuppression=nonmax, type=cv2.FAST_FEATURE_DETECTOR_TYPE_9_16)
    kps = fast.detect(gray, None)
    # Spread them with a grid and cap
    kps = grid_keep_top_k(kps, max_total=max_kpts, grid=(20, 15), image_shape=gray.shape)
    return kps

def compute_descriptors(gray, keypoints, prefer_brief=False):
    """
    Use ORB's descriptor (rotated BRIEF). If you have opencv-contrib, you could
    swap to BRIEF here (cv2.xfeatures2d.BriefDescriptorExtractor_create), but
    ORB descriptors are ideal and built-in.
    """
    orb = cv2.ORB_create(nfeatures=2000, edgeThreshold=19, patchSize=31, fastThreshold=0)  # we'll supply kps
    keypoints, desc = orb.compute(gray, keypoints)
    return keypoints, desc

def match_descriptors(desc1, desc2, ratio=0.75):
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    raw = bf.knnMatch(desc1, desc2, k=2)
    good = []
    for m, n in raw:
        if m.distance < ratio * n.distance:
            good.append(m)
    return good

def estimate_homography(kps1, kps2, matches, reproj_thresh=2.0):
    if len(matches) < 4:
        return None, None
    pts1 = np.float32([kps1[m.queryIdx].pt for m in matches])
    pts2 = np.float32([kps2[m.trainIdx].pt for m in matches])
    H, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, reproj_thresh)
    inliers = mask.ravel().tolist() if mask is not None else None
    return H, inliers

def draw_results(img1, img2, kps1, kps2, matches, inliers, H):
    # Draw matches
    if inliers is not None:
        inlier_matches = [m for m, keep in zip(matches, inliers) if keep]
    else:
        inlier_matches = matches
    matched_vis = cv2.drawMatches(img1, kps1, img2, kps2, inlier_matches, None,
                                  matchColor=(0,255,0), singlePointColor=(255,0,0),
                                  flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    # Draw projected outline of img1 onto img2 using H
    if H is not None:
        h, w = img1.shape[:2]
        box = np.float32([[0,0],[w,0],[w,h],[0,h]]).reshape(-1,1,2)
        proj = cv2.perspectiveTransform(box, H)
        proj_int = np.int32(proj + np.array([[[w,0]]], dtype=np.int32))  # shift because drawMatches concatenates
        cv2.polylines(matched_vis, [proj_int], True, (255,0,255), 3, cv2.LINE_AA)
    return matched_vis

def main():
    ap = argparse.ArgumentParser(description="FAST + ORB demo: detection, description, matching, homography.")
    ap.add_argument("--image", type=str, default="", help="Path to an image (optional).")
    ap.add_argument("--max-kpts", type=int, default=500, help="Max keypoints per image.")
    ap.add_argument("--fast-thresh", type=int, default=16, help="FAST corner threshold.")
    ap.add_argument("--ratio", type=float, default=0.75, help="Lowe ratio for KNN match.")
    ap.add_argument("--reproj", type=float, default=2.0, help="RANSAC reprojection threshold (px).")
    ap.add_argument("--save", type=str, default="output_matches.png", help="Where to save visualization.")
    args = ap.parse_args()

    if args.image and os.path.exists(args.image):
        img0 = cv2.imread(args.image, cv2.IMREAD_GRAYSCALE)
        if img0 is None:
            print("Failed to read image; falling back to checkerboard.")
            img0 = make_checkerboard()
    else:
        img0 = make_checkerboard()

    img1 = img0
    img2 = synthetic_transform(img0)

    # Detect keypoints with FAST
    kps1 = detect_fast(img1, threshold=args.fast_thresh, nonmax=True, max_kpts=args.max_kpts)
    kps2 = detect_fast(img2, threshold=args.fast_thresh, nonmax=True, max_kpts=args.max_kpts)

    # Describe with ORB's BRIEF (rotated)
    kps1, desc1 = compute_descriptors(img1, kps1)
    kps2, desc2 = compute_descriptors(img2, kps2)

    if desc1 is None or desc2 is None or len(kps1) == 0 or len(kps2) == 0:
        print("No descriptors/keypoints produced. Try lowering --fast-thresh.")
        sys.exit(1)

    # Match
    good = match_descriptors(desc1, desc2, ratio=args.ratio)

    # Homography
    H, inliers = estimate_homography(kps1, kps2, good, reproj_thresh=args.reproj)

    # Report
    inlier_count = sum(inliers) if inliers is not None else 0
    print(f"Keypoints: img1={len(kps1)} img2={len(kps2)}")
    print(f"Good matches (ratio {args.ratio:.2f}): {len(good)}")
    print(f"Inliers: {inlier_count} {'(homography found)' if H is not None else '(no H)'}")

    # Visualize
    vis = draw_results(cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR),
                       cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR),
                       kps1, kps2, good, inliers, H)
    cv2.imwrite(args.save, vis)
    print(f"Saved visualization to: {args.save}")

    # Optional preview (comment out if running headless)
    try:
        cv2.imshow("FAST+ORB matches", vis)
        cv2.waitKey(0)
    except Exception:
        pass

if __name__ == "__main__":
    main()
