"""Planar motion for a level camera: image right=-body Y, down=-body X."""
import math

import cv2
import numpy as np


def estimate_image_motion(old, new, fx, fy, cx, cy):
    """
    Fit previous->current floor motion about the calibrated principal point.

    Fit in equal focal-length coordinates so non-square pixels do not turn
    a physical rotation into a shear. Return pixel motion, image angle, mask.
    """
    focal = math.sqrt(fx * fy)
    scale = np.array([focal / fx, focal / fy])
    center = np.array([cx, cy])
    a = np.asarray((old - center) * scale, dtype=np.float32)
    b = np.asarray((new - center) * scale, dtype=np.float32)
    matrix, inliers = cv2.estimateAffinePartial2D(
        a, b, method=cv2.RANSAC, ransacReprojThreshold=2.0)
    if matrix is None or inliers is None or not np.isfinite(matrix).all():
        return None
    translation = matrix[:, 2] / scale
    angle = math.atan2(matrix[1, 0], matrix[0, 0])
    return float(translation[0]), float(translation[1]), angle, inliers


def image_to_reference(du, dv, theta, height, fx, fy, camera_x, camera_y,
                       reference_x=0.0, reference_y=0.0):
    """
    Return reference-origin displacement in the PREVIOUS body axes.

    theta is image rotation in (u right, v down). The downward camera's
    axis mapping reverses handedness, so body yaw is +theta. The floor
    transform in body coordinates is R(-theta), g. Inverting gives camera
    motion -R(theta)g. Subtract (R(theta)-I)(camera-reference).
    All mounting/reference offsets are expressed in body_link coordinates.
    """
    gx, gy = -dv * height / fy, -du * height / fx
    c, s = math.cos(theta), math.sin(theta)
    rx, ry = camera_x - reference_x, camera_y - reference_y
    return (-(c * gx - s * gy) - ((c - 1) * rx - s * ry),
            -(s * gx + c * gy) - (s * rx + (c - 1) * ry))


def integrate_pose(x, y, yaw, dx, dy, theta):
    """Compose finite SE(2) increments, without a midpoint approximation."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * dx - s * dy, y + s * dx + c * dy,
            math.atan2(math.sin(yaw + theta), math.cos(yaw + theta)))
