"""Explicit reference -> live image geometry and calibrated ground projection.

World frame: local Cartesian, z up. At zero yaw a nadir camera maps image right
to world +x and image up to world +y. Height is CAMERA height above a flat z=0
ground plane. camera_to_world includes the camera mount and aircraft attitude.
"""
from dataclasses import dataclass

import cv2
import numpy as np

NADIR_CAMERA_TO_WORLD = np.diag([1.0, -1.0, -1.0])


def checked_rotation(camera_to_world):
    R = np.asarray(camera_to_world, dtype=float)
    if R.shape != (3, 3) or not np.isfinite(R).all():
        raise ValueError("Missing/invalid camera-to-world rotation")
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(R), 1.0, atol=1e-5):
        raise ValueError("Camera-to-world matrix must be a proper rotation")
    return R


def project_point(H, point):
    H = np.asarray(H, dtype=float)
    p = np.asarray(point, dtype=float)
    if H.shape != (3, 3) or p.shape != (2,) or not np.isfinite(H).all() or not np.isfinite(p).all():
        raise ValueError("Invalid homography or point")
    q = H @ np.r_[p, 1.0]
    if not np.isfinite(q).all() or abs(q[2]) < 1e-9:
        raise ValueError("Point projects at infinity")
    return q[:2] / q[2]


def local_scale(H, point):
    """Area-equivalent reference -> live scale at a point, including perspective."""
    H = np.asarray(H, dtype=float)
    q = project_point(H, point)
    denominator = H[2] @ np.r_[point, 1.0]
    J = (H[:2, :2] - np.outer(q, H[2, :2])) / denominator
    singular = np.linalg.svd(J, compute_uv=False)
    if np.linalg.det(J) <= 0 or singular[-1] < 1e-8:
        raise ValueError("Reflected or degenerate local projection")
    return float(np.sqrt(np.prod(singular))), float(singular[0] / singular[-1])


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple = ()

    def __post_init__(self):
        values = [self.fx, self.fy, self.cx, self.cy, *self.distortion]
        if self.width <= 0 or self.height <= 0 or not np.isfinite(values).all() or min(self.fx, self.fy) <= 0:
            raise ValueError("Invalid camera calibration")
        if len(self.distortion) not in (0, 4, 5, 8, 12, 14):
            raise ValueError("Use OpenCV pinhole distortion coefficients")

    @property
    def matrix(self):
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], float)

    def resized(self, width, height):
        sx, sy = width / self.width, height / self.height
        return CameraModel(width, height, self.fx*sx, self.fy*sy, self.cx*sx, self.cy*sy)

    def ground_offset(self, rectified_pixel, height_m, camera_to_world):
        """Horizontal target displacement from camera, in local world x/y meters."""
        if self.distortion and np.any(self.distortion):
            raise ValueError("Projection expects a rectified camera model")
        R = checked_rotation(camera_to_world)
        if not np.isfinite(height_m) or height_m <= 0:
            raise ValueError("Current camera height must be positive")
        p = np.asarray(rectified_pixel, dtype=float)
        if p.shape != (2,) or not np.isfinite(p).all():
            raise ValueError("Invalid target pixel")
        ray = R @ np.linalg.solve(self.matrix, np.r_[p, 1.0])
        # Reject rays within ~5.7 degrees of the horizon: very poorly conditioned.
        if ray[2] / np.linalg.norm(ray) >= -0.1:
            raise ValueError("Target ray is near or above the horizon")
        return (-float(height_m) / ray[2]) * ray[:2]

    def nadir_pixel(self, camera_to_world):
        """Rectified image location of the ground directly below the camera."""
        ray = checked_rotation(camera_to_world).T @ np.array([0.,0.,-1.])
        if ray[2] <= 1e-9:
            raise ValueError("Ground nadir is behind the camera")
        p = self.matrix @ ray
        return p[:2]/p[2]


def plane_homography(camera, reference_position, reference_rotation, live_position, live_rotation):
    """Known-answer z=0 plane projection for simulations (world -> camera inverse)."""
    def plane_matrix(position, rotation):
        Rt = np.asarray(rotation, float).T
        t = -Rt @ np.asarray(position, float)
        return camera.matrix @ np.column_stack((Rt[:, 0], Rt[:, 1], t))
    return plane_matrix(live_position, live_rotation) @ np.linalg.inv(
        plane_matrix(reference_position, reference_rotation))


def camera_to_ground_homography(camera, position, camera_to_world):
    """Image pixels -> world x/y on z=0, for synthetic camera rendering."""
    Rt = checked_rotation(camera_to_world).T
    position = np.asarray(position, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all() or position[2] <= 0:
        raise ValueError('Camera position must be finite and above ground')
    plane = camera.matrix @ np.column_stack((Rt[:, 0], Rt[:, 1], -Rt @ position))
    return np.linalg.inv(plane)
