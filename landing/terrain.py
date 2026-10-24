"""Coordinate-seeded gravel terrain, generated in bounded cached chunks."""
from functools import lru_cache
import math

import cv2
import numpy as np

from .geometry import NADIR_CAMERA_TO_WORLD, camera_to_ground_homography

GROUND_WIDTH_M = 256.
PIXELS_PER_M = 192.
CHUNK_SIZE = 1024


def _hash(x, y, seed):
    """Stable unsigned hash of world lattice coordinates."""
    x, y = np.asarray(x).astype(np.uint64), np.asarray(y).astype(np.uint64)
    value = x*np.uint64(73856093) ^ y*np.uint64(19349663) ^ np.uint64(seed % (2**64))
    value = (value ^ (value >> np.uint64(16))) * np.uint64(2246822519)
    return value ^ (value >> np.uint64(13))


@lru_cache(maxsize=48)
def ground_chunk(seed, cx, cy):
    """Unique read-only chunk; soil and crossing stones agree at boundaries.

    Cache holds at most 144 MiB of RGB pixels regardless of world size.
    """
    size = CHUNK_SIZE
    ox, oy = cx*size, cy*size
    shade = np.full((size, size), 135, np.float32)
    for spacing, weight in ((2048, .35), (512, .25), (128, .2)):
        xs = (np.arange(size, dtype=np.float32)+ox)/spacing
        ys = (np.arange(size, dtype=np.float32)+oy)/spacing
        ix, iy = np.floor(xs).astype(int), np.floor(ys).astype(int)
        fx, fy = xs-ix, ys-iy
        def lattice(dx, dy):
            return (_hash(ix[None, :]+dx, iy[:, None]+dy, seed) % 180).astype(np.float32)+40
        top = lattice(0, 0)*(1-fx) + lattice(1, 0)*fx
        bottom = lattice(0, 1)*(1-fx) + lattice(1, 1)*fx
        noise = top*(1-fy[:, None]) + bottom*fy[:, None]
        shade = shade*(1-weight)+noise*weight
    rng = np.random.default_rng([seed % (2**32), cx % (2**32), cy % (2**32)])
    shade = np.uint8(np.clip(shade*.75+rng.integers(75, 190, (size, size))*.25, 0, 255))
    ground = cv2.merge((cv2.subtract(shade, 25), cv2.subtract(shade, 9), shade))
    # Generate stones in world cells, including neighbors whose stones cross
    # this chunk. Every chunk paints intersecting cells in the same order.
    for cell_y in range((oy-18)//256, (oy+size+18)//256+1):
        for cell_x in range((ox-18)//256, (ox+size+18)//256+1):
            rng = np.random.default_rng([seed % (2**32), cell_x % (2**32), cell_y % (2**32), 37])
            centers = rng.integers(0, 256, (112, 2))
            radii = rng.integers(2, 15, (112, 2))
            colors = rng.integers(65, 215, 112)
            angles = rng.integers(0, 180, 112)
            for (x, y), (rx, ry), color, angle in zip(centers, radii, colors, angles):
                x, y = int(x)+cell_x*256-ox, int(y)+cell_y*256-oy
                rx, ry, color, angle = map(int, (rx, ry, color, angle))
                if x+rx+3 < 0 or x-rx-3 >= size or y+ry+3 < 0 or y-ry-3 >= size:
                    continue
                cv2.ellipse(ground, (x+2, y+2), (rx, ry), angle, 0, 360, (58, 66, 72), -1, cv2.LINE_AA)
                cv2.ellipse(ground, (x, y), (rx, ry), angle, 0, 360,
                            (max(0, color-18), max(0, color-6), color), -1, cv2.LINE_AA)
                if rx > 6:
                    cv2.line(ground, (x-rx//3, y-ry//3), (x+rx//3, y-ry//3),
                             (min(255, color+12), min(255, color+20), min(255, color+25)), 1, cv2.LINE_AA)
    ground.flags.writeable = False
    return ground


class ProceduralGround:
    def __init__(self, seed=7, width_m=GROUND_WIDTH_M):
        if not math.isfinite(width_m) or not 32 <= width_m <= 4096:
            raise ValueError('Ground width must be between 32 and 4096 meters')
        self.seed, self.width_m = int(seed), float(width_m)

    def render(self, position, camera, rotation=NADIR_CAMERA_TO_WORLD):
        matrix = np.diag([PIXELS_PER_M, -PIXELS_PER_M, 1.]) @ camera_to_ground_homography(camera, position, rotation)
        corners = np.array([[0, 0, 1], [camera.width-1, 0, 1],
                            [camera.width-1, camera.height-1, 1], [0, camera.height-1, 1]]) @ matrix.T
        corners = corners[:, :2]/corners[:, 2:]
        x0, y0 = np.floor(corners.min(axis=0)).astype(int)-1
        x1, y1 = np.ceil(corners.max(axis=0)).astype(int)+2
        patch = np.full((y1-y0, x1-x0, 3), 65, np.uint8)
        limit = self.width_m*PIXELS_PER_M/2
        for cy in range(y0//CHUNK_SIZE, (y1-1)//CHUNK_SIZE+1):
            for cx in range(x0//CHUNK_SIZE, (x1-1)//CHUNK_SIZE+1):
                left = max(x0, cx*CHUNK_SIZE, math.ceil(-limit))
                right = min(x1, (cx+1)*CHUNK_SIZE, math.floor(limit))
                top = max(y0, cy*CHUNK_SIZE, math.ceil(-limit))
                bottom = min(y1, (cy+1)*CHUNK_SIZE, math.floor(limit))
                if left >= right or top >= bottom:
                    continue
                chunk = ground_chunk(self.seed, cx, cy)
                patch[top-y0:bottom-y0, left-x0:right-x0] = chunk[
                    top-cy*CHUNK_SIZE:bottom-cy*CHUNK_SIZE,
                    left-cx*CHUNK_SIZE:right-cx*CHUNK_SIZE]
        matrix = np.array([[1, 0, -x0], [0, 1, -y0], [0, 0, 1]]) @ matrix
        return cv2.warpPerspective(patch, matrix, (camera.width, camera.height),
                              flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
