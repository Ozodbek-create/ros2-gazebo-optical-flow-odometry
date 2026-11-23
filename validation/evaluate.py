"""Continuous replay, SE(2) ATE/RPE, controlled alternatives and plots."""
import argparse
import json
from pathlib import Path
import time

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from optical_flow_odometry.motion_geometry import image_to_reference, integrate_pose
from optical_flow_odometry.tracking import IntervalGuard, Tracker


def local(poses):
    p = poses.copy()
    yaw = p[0, 2]
    c, s = np.cos(yaw), np.sin(yaw)
    p[:, :2] = (p[:, :2] - p[0, :2]) @ np.array([[c, -s], [s, c]])
    p[:, 2] -= yaw
    return p


def difference(a, b):
    d = b-a
    c, s = np.cos(a[:, 2]), np.sin(a[:, 2])
    return np.column_stack((c*d[:, 0]+s*d[:, 1], -s*d[:, 0]+c*d[:, 1], d[:, 2]))


def run(data, corners, fb, corruption=None):
    tracker, guard = Tracker(corners, fb), IntervalGuard()
    k = data['k'][[0, 4, 2, 5]]
    poses, records, runtime = [], [], []
    pose = np.zeros(3)
    for i, (image, stamp) in enumerate(zip(data['frames'], data['stamps'])):
        if corruption == 'blank' and 100 <= i < 130:
            image = np.zeros_like(image)
        if corruption == 'occluded' and 100 <= i < 160:
            image = image.copy()
            image[:, :160] = 0
        if corruption == 'drop' and i % 5 == 0 and i > 0:
            poses.append(pose.copy())
            continue
        start = time.perf_counter()
        m = tracker.process(image, round(float(stamp)*1e9), k)
        runtime.append((time.perf_counter()-start)*1000)
        accepted, reason = guard.admit(m)
        m['admission'] = reason
        if accepted:
            dx, dy = image_to_reference(m['du'], m['dv'], m['dtheta'], .085,
                                        *k[:2], -.15, 0.)
            new = integrate_pose(*pose, dx, dy, m['dtheta'])
            pose = np.array([*new[:2], pose[2]+m['dtheta']])
        poses.append(pose.copy())
        records.append(m)
    return np.array(poses), records, runtime, guard


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--output', default='validation_output')
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    data = np.load(args.dataset)
    cv2.setNumThreads(1)
    times = data['stamps']
    if len(times) < 2 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('Evaluation requires strictly increasing finite image timestamps')
    raw = data['truth'][np.argsort(data['truth'][:, 0])].copy()
    raw[:, 3] = np.unwrap(raw[:, 3])
    # Never extrapolate ground truth; restrict all metrics to covered timestamps.
    covered = (times >= raw[0, 0]) & (times <= raw[-1, 0])
    truth = local(np.column_stack([np.interp(times, raw[:, 0], raw[:, i]) for i in (1, 2, 3)]))
    distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(truth[:, :2], axis=0), axis=1))]
    summary = {'dataset': str(Path(args.dataset).resolve()), 'duration_s': float(times[-1]-times[0]),
               'body_path_length_m': float(distance[-1]), 'covered_frames': int(covered.sum()), 'variants': {}}
    variants = [('baseline200', 200, False, None), ('fb200', 200, True, None),
                ('sparse80', 80, False, None), ('dense400', 400, False, None),
                ('blank30', 200, False, 'blank'), ('occluded60', 200, False, 'occluded'),
                ('drop_every5', 200, False, 'drop')]
    baseline = None
    for name, corners, fb, corruption in variants:
        cv2.setRNGSeed(0)
        poses, records, runtime, guard = run(data, corners, fb, corruption)
        error = poses-truth
        error[:, 2] = np.arctan2(np.sin(error[:, 2]), np.cos(error[:, 2]))
        poserr = np.linalg.norm(error[:, :2], axis=1)
        # RPE over approximately 1 second, measured as relative SE(2) motion.
        j = np.searchsorted(times, times+1.)
        i = np.flatnonzero((j < len(times)) & covered)
        i = i[covered[j[i]]]
        rel = difference(poses[i], poses[j[i]]) - difference(truth[i], truth[j[i]])
        rel[:, 2] = np.arctan2(np.sin(rel[:, 2]), np.cos(rel[:, 2]))
        last = np.flatnonzero(covered)[-1]
        row = dict(ate_rmse_m=float(np.sqrt(np.mean(poserr[covered]**2))),
                   final_position_error_m=float(poserr[last]),
                   final_yaw_error_deg=float(np.degrees(error[last, 2])),
                   yaw_rmse_deg=float(np.sqrt(np.mean(np.degrees(error[covered, 2])**2))),
                   rpe_1s_position_rmse_m=float(np.sqrt(np.mean(np.sum(rel[:, :2]**2, axis=1)))),
                   rpe_1s_yaw_rmse_deg=float(np.sqrt(np.mean(np.degrees(rel[:, 2])**2))),
                   valid_fraction=float(sum(m['valid'] for m in records)/max(1, len(records)-1)),
                   accepted=guard.accepted, rejected=guard.rejected, pose_incomplete=guard.incomplete,
                   mean_processing_ms=float(np.mean(runtime)), p95_processing_ms=float(np.percentile(runtime, 95)))
        row['reasons'] = {reason: sum(m['reason'] == reason for m in records)
                          for reason in sorted(set(m['reason'] for m in records))}
        summary['variants'][name] = row
        (out / (name+'_records.json')).write_text(json.dumps(records))
        if name == 'baseline200':
            baseline = (poses, error, poserr)
    poses, error, poserr = baseline
    phase_file = Path(args.dataset).with_name('phases.json')
    summary['motion_types'] = []
    if phase_file.exists():
        for name, start, end, *_ in json.loads(phase_file.read_text()):
            ind = np.flatnonzero((times >= start) & (times <= end) & covered)
            if len(ind) < 2:
                continue
            a, b = ind[0], ind[-1]
            rel = difference(poses[[a]], poses[[b]]) - difference(truth[[a]], truth[[b]])
            summary['motion_types'].append(dict(name=name,
                relative_position_error_m=float(np.linalg.norm(rel[0, :2])),
                relative_yaw_error_deg=float(np.degrees(rel[0, 2])),
                accumulated_position_error_m=float(poserr[b])))
    (out / 'summary.json').write_text(json.dumps(summary, indent=2))
    np.savez_compressed(out / 'trajectory.npz', times=times, truth=truth, poses=poses,
                        position_error=poserr, yaw_error=error[:, 2], distance=distance, covered=covered)
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    ax = axes[0, 0]
    ax.plot(truth[:, 0], truth[:, 1], label='Gazebo world pose')
    ax.plot(poses[:, 0], poses[:, 1], label='Optical body origin')
    ax.set(xlabel='x (m)', ylabel='y (m)', title='Continuous trajectory', aspect='equal')
    ax.legend()
    axes[0, 1].plot((times-times[0])[covered], poserr[covered]*1000)
    axes[0, 1].set(xlabel='elapsed simulation time (s)', ylabel='position error (mm)')
    axes[1, 0].plot((times-times[0])[covered], np.degrees(error[covered, 2]))
    axes[1, 0].set(xlabel='elapsed simulation time (s)', ylabel='yaw error (degrees)')
    axes[1, 1].plot(distance[covered], poserr[covered]*1000)
    axes[1, 1].set(xlabel='body path length (m)', ylabel='position error (mm)')
    fig.tight_layout()
    fig.savefig(out / 'trajectory_errors.png', dpi=150)
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
