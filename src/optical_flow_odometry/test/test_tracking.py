"""Exercise failures, interval correctness and reacquisition deterministically."""
import numpy as np
from optical_flow_odometry.tracking import IntervalGuard, Tracker
import pytest


def record(a=0, b=33_000_000):
    """Create a fully specified synthetic accepted record."""
    return {'version': 1, 'start_ns': a, 'end_ns': b, 'valid': True, 'reason': 'ok',
            'du': 1., 'dv': 0., 'dtheta': 0., 'tracked': 20, 'inliers': 18,
            'inlier_ratio': .9, 'residual_px': .1, 'scale': 1., 'coverage': .3}


@pytest.mark.parametrize('field,value', [
    ('du', float('nan')), ('dtheta', float('inf')), ('du', 60.),
    ('inliers', 3), ('inlier_ratio', .2), ('residual_px', 2.),
    ('coverage', .001), ('scale', 1.2), ('end_ns', -1), ('version', 2)])
def test_invalid(field, value):
    m = record()
    m[field] = value
    guard = IntervalGuard()
    assert not guard.admit(m)[0]
    assert guard.incomplete


def test_duplicates_and_loss_recovery():
    guard = IntervalGuard()
    assert guard.admit(record())[0]
    assert not guard.admit(record())[0]
    assert not guard.incomplete
    failed = record(33_000_000, 66_000_000)
    failed.update(valid=False, reason='insufficient_texture')
    assert not guard.admit(failed)[0]
    assert guard.admit(record(66_000_000, 99_000_000))[0]
    assert guard.incomplete
    assert guard.accepted == 2


def test_dropped_messages_use_local_interval():
    guard = IntervalGuard()
    assert guard.admit(record())[0]
    assert guard.admit(record(66_000_000, 99_000_000))[0]
    assert guard.incomplete
    assert not guard.admit(record(99_000_000, 400_000_000))[0]
    assert not guard.admit(record(50_000_000, 120_000_000))[0]


def test_blank_frames_and_recovery():
    tracker = Tracker()
    rng = np.random.default_rng(10)
    textured = rng.integers(0, 255, (240, 320), dtype=np.uint8)
    blank = np.zeros_like(textured)
    k = (160., 160., 160., 120.)
    assert not tracker.process(blank, 0, k)['valid']
    assert tracker.process(blank, 33_000_000, k)['reason'] == 'insufficient_texture'
    assert not tracker.process(textured, 66_000_000, k)['valid']
    assert tracker.process(textured, 99_000_000, k)['valid']
    assert tracker.process(textured, 99_000_000, k)['reason'] == 'duplicate'
    assert tracker.process(textured, 400_000_000, k)['reason'] == 'frame_gap'
    assert tracker.process(textured, 433_000_000, k)['valid']
    assert tracker.process(textured, 1, k)['reason'] == 'clock_reset'
    assert tracker.process(textured, 33_000_001, k)['valid']


def test_bad_schema_and_calibration():
    assert not IntervalGuard().admit({})[0]
    tracker = Tracker()
    image = np.zeros((240, 320), dtype=np.uint8)
    assert tracker.process(image, 0, None)['reason'] == 'no_calibration'
