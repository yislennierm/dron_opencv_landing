from dataclasses import dataclass

import cv2
import numpy as np

from .geometry import CameraModel


@dataclass
class Features:
    points: np.ndarray
    descriptors: np.ndarray | None
    shape: tuple


class Extractor:
    def __init__(self, max_features=600, max_width=640, max_height=480, camera=None):
        self.max_features = max_features
        self.max_width, self.max_height = max_width, max_height
        self.camera: CameraModel | None = camera
        self._rectification = None
        self._mask = None
        self.orb = cv2.ORB_create(nfeatures=max_features*2, edgeThreshold=19,
                                 patchSize=31, fastThreshold=16)

    def prepare(self, image):
        if image is None or image.size == 0 or image.dtype != np.uint8:
            raise ValueError("Expected a nonempty uint8 grayscale or BGR frame")
        if image.ndim == 3 and image.shape[2] == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        elif image.ndim == 2:
            gray = image
        else:
            raise ValueError("Expected grayscale or BGR frame")
        h, w = gray.shape
        if self.camera and (w, h) != (self.camera.width, self.camera.height):
            raise ValueError("Frame dimensions do not match calibration; recalibrate or scale it explicitly")
        scale = min(1.0, self.max_width/w, self.max_height/h)
        size = (max(1, round(w*scale)), max(1, round(h*scale)))
        gray = cv2.resize(gray, size, interpolation=cv2.INTER_AREA) if size != (w,h) else gray.copy()
        camera = self.camera.resized(*size) if self.camera else None
        if self.camera and self.camera.distortion and np.any(self.camera.distortion):
            if self._rectification is None:
                maps = cv2.initUndistortRectifyMap(camera.matrix, np.asarray(self.camera.distortion, float),
                    None, camera.matrix, size, cv2.CV_32FC1)
                valid = cv2.remap(np.full_like(gray,255), *maps, cv2.INTER_LINEAR)
                # Synthetic black rectification borders must not become stable
                # scene features. Leave a patch margin around invalid pixels.
                self._mask = cv2.erode(np.uint8(valid == 255)*255,np.ones((41,41),np.uint8))
                self._rectification = maps
            gray = cv2.remap(gray, *self._rectification, cv2.INTER_LINEAR)
        return gray, camera

    def extract(self, gray):
        keypoints, descriptors = self.orb.detectAndCompute(gray, self._mask)
        if descriptors is None:
            return Features(np.empty((0,2), np.float32), None, gray.shape)
        # Round-robin grid selection preserves oriented, multiscale ORB keypoints
        # while preventing a high-contrast corner from taking the entire budget.
        h, w = gray.shape
        cells = [[] for _ in range(8*6)]
        for i, kp in enumerate(keypoints):
            x, y = min(7, int(kp.pt[0]*8/w)), min(5, int(kp.pt[1]*6/h))
            cells[y*8+x].append(i)
        for cell in cells:
            cell.sort(key=lambda i: keypoints[i].response, reverse=True)
        chosen = []
        depth = 0
        while len(chosen) < self.max_features:
            layer = [cell[depth] for cell in cells if depth < len(cell)]
            if not layer:
                break
            chosen.extend(layer[:self.max_features-len(chosen)])
            depth += 1
        return Features(np.float32([keypoints[i].pt for i in chosen]), descriptors[chosen].copy(), gray.shape)


def mutual_ratio_matches(reference, live, ratio=0.75):
    if reference.descriptors is None or live.descriptors is None:
        return []
    if min(len(reference.descriptors), len(live.descriptors)) < 2:
        return []
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    def filtered(a, b):
        pairs = matcher.knnMatch(a, b, k=2)
        return {pair[0].queryIdx: pair[0] for pair in pairs if len(pair) == 2
                and pair[0].distance < ratio*pair[1].distance}
    forward = filtered(reference.descriptors, live.descriptors)
    reverse = filtered(live.descriptors, reference.descriptors)
    return [m for i,m in forward.items() if m.trainIdx in reverse and reverse[m.trainIdx].trainIdx == i]
