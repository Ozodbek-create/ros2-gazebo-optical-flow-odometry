"""Tracking measurements and explicit interval admission, independent of ROS."""
import math

import cv2
import numpy as np


def quality_ok(m):
    """Apply engineering gates; these are not probability calibration."""
    return (m['tracked'] >= 12 and m['inliers'] >= 8
            and .5 <= m['inlier_ratio'] <= 1
            and 0 <= m['residual_px'] <= 1.5
            and .95 <= m['scale'] <= 1.05
            and m['coverage'] >= .01)


class IntervalGuard:
    """Reject malformed/repeated intervals and latch unrecoverable pose gaps."""

    def __init__(self):
        self.end = None
        self.incomplete = False
        self.accepted = 0
        self.rejected = 0

    def admit(self, m):
        """Return (accepted, reason); never infer a missing displacement."""
        try:
            if m['version'] != 1 or type(m['valid']) is not bool:
                raise ValueError('schema')
            a, b = m['start_ns'], m['end_ns']
            if type(a) is not int or type(b) is not int or min(a, b) < 0:
                raise ValueError('timestamp')
            if m.get('reason') == 'clock_reset' and not m['valid']:
                self.end = b
                self.incomplete = True
                self.rejected += 1
                return False, 'clock_reset'
            if self.end is not None and b <= self.end:
                self.rejected += 1
                return False, 'duplicate_or_out_of_order'
            if not a <= b or b - a > 200_000_000:
                raise ValueError('interval')
            if self.end is not None and a < self.end:
                raise ValueError('overlapping_interval')
            if self.end is not None and a != self.end:
                self.incomplete = True
            self.end = b
            if not m['valid']:
                if a != b:
                    self.incomplete = True
                self.rejected += 1
                return False, m.get('reason', 'invalid')
            values = [m[k] for k in ('du', 'dv', 'dtheta', 'inlier_ratio',
                                     'residual_px', 'scale', 'coverage')]
            if not all(math.isfinite(v) for v in values):
                raise ValueError('nonfinite')
            if a == b or not quality_ok(m):
                raise ValueError('quality')
            if (type(m['tracked']) is not int or type(m['inliers']) is not int
                    or not 0 <= m['inliers'] <= m['tracked']
                    or abs(m['inlier_ratio'] - m['inliers']/m['tracked']) > 1e-6):
                raise ValueError('inconsistent_counts')
            if abs(m['du']) > 50 or abs(m['dv']) > 50 or abs(m['dtheta']) > .25:
                raise ValueError('motion_limit')
        except (KeyError, TypeError, ValueError, OverflowError):
            self.incomplete = True
            self.rejected += 1
            return False, 'malformed_or_rejected'
        self.accepted += 1
        return True, 'accepted'


class Tracker:
    """Shi-Tomasi/LK/RANSAC with measured residuals and explicit failures."""

    def __init__(self, max_corners=200, forward_backward=False):
        self.previous = None
        self.stamp = None
        self.max_corners = max_corners
        self.forward_backward = forward_backward

    def process(self, gray, stamp, intrinsics):
        """Return one finite JSON-compatible record per input frame."""
        old_stamp = self.stamp
        m = {'version': 1, 'start_ns': stamp if old_stamp is None else old_stamp,
             'end_ns': stamp, 'valid': False, 'reason': 'initializing', 'du': 0., 'dv': 0.,
             'dtheta': 0., 'detected': 0, 'tracked': 0, 'inliers': 0, 'inlier_ratio': 0.,
             'residual_px': 0., 'scale': 1., 'coverage': 0.}
        if old_stamp is not None and stamp == old_stamp:
            m['reason'] = 'duplicate'
            return m
        previous = self.previous
        self.previous, self.stamp = gray.copy(), stamp
        if old_stamp is not None and stamp < old_stamp:
            m.update(start_ns=stamp, reason='clock_reset')
            return m
        if old_stamp is not None and stamp - old_stamp > 200_000_000:
            m['reason'] = 'frame_gap'
            return m
        if intrinsics is None:
            m['reason'] = 'no_calibration'
            return m
        if previous is None or previous.shape != gray.shape:
            return m
        try:
            p0 = cv2.goodFeaturesToTrack(previous, self.max_corners, .01, 7,
                                         blockSize=7)
            m['detected'] = 0 if p0 is None else len(p0)
            if m['detected'] < 12:
                m['reason'] = 'insufficient_texture'
                return m
            params = {'winSize': (21, 21), 'maxLevel': 3,
                      'criteria': (cv2.TERM_CRITERIA_EPS |
                                   cv2.TERM_CRITERIA_COUNT, 30, .01)}
            p1, status, _ = cv2.calcOpticalFlowPyrLK(
                previous, gray, p0, None, **params)
            if p1 is None or status is None:
                m['reason'] = 'tracking_failed'
                return m
            good = (status.ravel() == 1) & np.isfinite(p1.reshape(-1, 2)).all(axis=1)
            if self.forward_backward:
                back, backward_status, _ = cv2.calcOpticalFlowPyrLK(
                    gray, previous, p1, None, **params)
                if back is None or backward_status is None:
                    good[:] = False
                else:
                    good &= backward_status.ravel() == 1
                    good &= np.linalg.norm((back-p0).reshape(-1, 2), axis=1) < .5
            old, new = p0.reshape(-1, 2)[good], p1.reshape(-1, 2)[good]
            m['tracked'] = len(old)
            if len(old) < 12:
                m['reason'] = 'insufficient_tracks'
                return m
            fx, fy, cx, cy = intrinsics
            scaling = np.array([math.sqrt(fx*fy)/fx, math.sqrt(fx*fy)/fy])
            a = ((old - [cx, cy]) * scaling).astype(np.float32)
            b = ((new - [cx, cy]) * scaling).astype(np.float32)
            matrix, mask = cv2.estimateAffinePartial2D(
                a, b, method=cv2.RANSAC, ransacReprojThreshold=2.)
            if matrix is None or mask is None or not np.isfinite(matrix).all():
                m['reason'] = 'fit_failed'
                return m
            keep = mask.ravel() == 1
            m['inliers'] = int(keep.sum())
            m['inlier_ratio'] = float(keep.mean())
            if not keep.any():
                m['reason'] = 'no_inliers'
                return m
            residual = (a @ matrix[:, :2].T + matrix[:, 2] - b) / scaling
            m['residual_px'] = float(np.sqrt(np.mean(np.sum(residual[keep]**2, axis=1))))
            m['scale'] = float(np.hypot(matrix[0, 0], matrix[1, 0]))
            h, w = gray.shape
            m['coverage'] = float(cv2.contourArea(cv2.convexHull(old[keep])) / (w*h))
            m['du'], m['dv'] = map(float, matrix[:, 2] / scaling)
            m['dtheta'] = math.atan2(matrix[1, 0], matrix[0, 0])
            m['valid'] = bool(quality_ok(m) and abs(m['du']) <= 50
                              and abs(m['dv']) <= 50 and abs(m['dtheta']) <= .25)
            m['reason'] = 'ok' if m['valid'] else 'quality_rejected'
        except (cv2.error, ValueError, OverflowError):
            m['valid'], m['reason'] = False, 'processing_error'
        return m
