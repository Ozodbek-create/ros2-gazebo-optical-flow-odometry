"""Independent pinhole projections exercise the production geometry pipeline."""
import math

import cv2
import numpy as np
from optical_flow_odometry.motion_geometry import (
    estimate_image_motion, image_to_reference, integrate_pose)
import pytest


def rotation(theta):
    return np.array([[math.cos(theta), -math.sin(theta)],
                     [math.sin(theta), math.cos(theta)]])


def project(points, body_xy, theta, camera, height, intrinsics):
    """Project fixed WORLD points from independently specified body poses."""
    body_points = (points - body_xy) @ rotation(theta)
    camera_points = body_points - camera
    fx, fy, cx, cy = intrinsics
    return np.column_stack((cx - fx * camera_points[:, 1] / height,
                            cy - fy * camera_points[:, 0] / height))


@pytest.mark.parametrize('translation,theta', [
    ((0., 0.), 0.), ((.008, 0.), 0.), ((0., -.006), 0.),
    ((0., 0.), .12), ((0., 0.), -.12), ((.008, -.004), .08)])
@pytest.mark.parametrize('intrinsics', [(160., 160., 160., 120.),
                                        (210., 170., 153., 117.)])
def test_projected_motion(translation, theta, intrinsics):
    rng = np.random.default_rng(45)
    camera = np.array([-.15, .017])
    ground = camera + rng.uniform(-.06, .06, (150, 2))
    old = project(ground, (0., 0.), 0., camera, .085, intrinsics)
    new = project(ground, translation, theta, camera, .085, intrinsics)
    # Deliberately mismatched tracks test RANSAC rather than perfect-only data.
    new[:15] = rng.uniform(0, 300, (15, 2))
    du, dv, angle, mask = estimate_image_motion(old, new, *intrinsics)
    measured = image_to_reference(du, dv, angle, .085, *intrinsics[:2], *camera)
    np.testing.assert_allclose(measured, translation, atol=2e-8)
    assert angle == pytest.approx(theta, abs=2e-7)
    assert np.count_nonzero(mask) >= 130


@pytest.mark.parametrize('theta', [.0095, -.0095, .2, -.2])
def test_axle_rotation_has_real_body_translation(theta):
    camera = np.array([-.15, 0.])
    axle = np.array([-.15, 0.])
    body_delta = axle - rotation(theta) @ axle
    rng = np.random.default_rng(3)
    ground = camera + rng.uniform(-.05, .05, (100, 2))
    k = (160., 160., 160., 120.)
    old = project(ground, (0., 0.), 0., camera, .085, k)
    new = project(ground, body_delta, theta, camera, .085, k)
    du, dv, angle, _ = estimate_image_motion(old, new, *k)
    body = image_to_reference(du, dv, angle, .085, 160., 160., *camera)
    axle_motion = image_to_reference(du, dv, angle, .085, 160., 160.,
                                     *camera, *axle)
    np.testing.assert_allclose(body, body_delta, atol=2e-8)
    np.testing.assert_allclose(axle_motion, 0., atol=2e-8)


def test_finite_pose_composition():
    pose = (.2, -.4, 1.1)
    delta = (.03, -.02, .15)
    expected = np.array(pose[:2]) + rotation(pose[2]) @ delta[:2]
    measured = integrate_pose(*pose, *delta)
    np.testing.assert_allclose(measured[:2], expected, atol=1e-15)
    assert measured[2] == pytest.approx(1.25)
    assert integrate_pose(0., 0., 3.1, 0., 0., .1)[2] == pytest.approx(3.2 - 2*math.pi)


@pytest.mark.parametrize('theta', [.01, -.01])
def test_rendered_rotation_lucas_kanade(theta):
    rng = np.random.default_rng(7)
    old = np.zeros((240, 320), dtype=np.uint8)
    for x, y in rng.integers([20, 20], [300, 220], size=(150, 2)):
        cv2.rectangle(old, (x-2, y-2), (x+2, y+2), 255, -1)
    r = rotation(theta)
    center = np.array([160., 120.])
    affine = np.column_stack((r, center - r @ center))
    new = cv2.warpAffine(old, affine, (320, 240))
    p0 = cv2.goodFeaturesToTrack(old, 200, .01, 7, blockSize=7)
    p1, status, _ = cv2.calcOpticalFlowPyrLK(
        old, new, p0, None, winSize=(21, 21), maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
    good = status.ravel() == 1
    du, dv, angle, _ = estimate_image_motion(
        p0.reshape(-1, 2)[good], p1.reshape(-1, 2)[good], 160., 160., 160., 120.)
    assert angle == pytest.approx(theta, abs=0.0002)
    assert math.hypot(du, dv) < .05
