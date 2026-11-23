"""Bounded live experiment; saves images and independent simulator/wheel poses."""
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3Stamped
from cv_bridge import CvBridge
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y),
                      1 - 2 * (q.y*q.y + q.z*q.z))


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='validation_output/capture')
    out = Path(parser.parse_args().output)
    out.mkdir(parents=True, exist_ok=True)
    if (out / 'live_capture.npz').exists():
        raise RuntimeError('Preserve existing capture; choose a fresh output path')
    rclpy.init()
    node = Node('odometry_bounded_validation')
    bridge = CvBridge()
    frames, stamps, truth, wheel, motion, phases = [], [], [], [], [], []
    calibration = []
    gz = GzNode()

    def true_pose(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == 'differential_drive_robot':
                truth.append([t, p.position.x, p.position.y, yaw(p.orientation),
                              p.position.z, p.orientation.x, p.orientation.y])

    assert gz.subscribe(Pose_V, '/world/default/dynamic_pose/info', true_pose)

    def image(msg):
        frames.append(bridge.imgmsg_to_cv2(msg, 'mono8').copy())
        stamps.append(seconds(msg.header.stamp))

    def odom(msg):
        p = msg.pose.pose
        wheel.append([seconds(msg.header.stamp), p.position.x, p.position.y,
                      yaw(p.orientation)])

    def flow(msg):
        motion.append([seconds(msg.header.stamp), msg.vector.x,
                       msg.vector.y, msg.vector.z])

    node.create_subscription(Image, '/camera/down/image_raw', image,
                             qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/camera/down/camera_info',
                             lambda m: calibration.append(list(m.k)),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', odom, 100)
    node.create_subscription(Vector3Stamped, '/optical_flow/image_motion', flow, 100)
    # Discover existing publishers before creating our own.
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=.05)
    if node.count_publishers('/cmd_vel'):
        raise RuntimeError('Another ROS command publisher is present; no motion permitted')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    deadline = time.monotonic() + 15
    while not (stamps and truth and wheel and calibration):
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() > deadline:
            raise RuntimeError('Missing input; no commands sent')
    from ament_index_python.packages import get_package_share_directory
    root_world = str(Path(get_package_share_directory('gazebo_differential_drive_robot')) / 'worlds/optical_flow.sdf')
    try:
        # Read the visual mark geometry, not just the large collision plane.
        import xml.etree.ElementTree as ET
        world = ET.parse(root_world).getroot()
        marks = [v for v in world.findall('.//visual')
                 if v.get('name', '').startswith('mark_')]
        xy = np.array([[float(x) for x in v.findtext('pose').split()[:2]] for v in marks])
        # Conservative inscribed bounds; 0.085*sqrt(1+.75**2) image footprint,
        # plus 4cm allowance for tracking delay and sliding.
        margin = .15
        lower, upper = xy.min(axis=0)+margin, xy.max(axis=0)-margin

        def camera_position(pose):
            return np.array(pose[1:3]) - .15*np.array([math.cos(pose[3]), math.sin(pose[3])])

        def inside(point):
            return bool(np.all(point > lower) and np.all(point < upper))

        sequence = [('stationary', 0., 0., 2.)]
        for trial, factor in enumerate([1., 1.5]):
            for repeat in range(2):
                for label, v, w in [('forward', .02, 0.), ('reverse', -.02, 0.),
                                    ('ccw', 0., .18), ('cw', 0., -.18),
                                    ('arc', .012, .15), ('reverse_arc', -.012, -.15)]:
                    sequence.append((f't{trial}_r{repeat}_{label}', v*factor, w*factor, 2.))
                sequence.append((f't{trial}_r{repeat}_stop', 0., 0., 1.))
        sequence.append(('final_stop', 0., 0., 2.))
        # Predict the complete camera/axle path before the first nonzero command.
        predicted = camera_position(truth[-1])
        heading = truth[-1][3]
        if not inside(predicted):
            raise RuntimeError(f'Camera outside conservative texture bounds: {predicted}')
        for _, v, w, duration in sequence:
            for _ in range(round(duration/.02)):
                predicted = predicted + .02*v*np.array([math.cos(heading+w*.01),
                                                       math.sin(heading+w*.01)])
                heading += w*.02
                if not inside(predicted):
                    raise RuntimeError('Predicted path exits conservative textured bounds')
        (out / 'preflight.json').write_text(json.dumps(dict(
            mark_lower=xy.min(axis=0).tolist(), mark_upper=xy.max(axis=0).tolist(),
            center_lower=lower.tolist(), center_upper=upper.tolist(),
            initial_camera=camera_position(truth[-1]).tolist(),
            initial_world_pose=truth[-1], sequence=sequence), indent=2))
        print('PREFLIGHT PASSED', lower, upper, flush=True)
        for name, v, w, duration in sequence:
            start = stamps[-1]
            phases.append([name, start, start + duration, v, w])
            print(name, start, flush=True)
            limit = time.monotonic() + 30
            while stamps[-1] - start < duration:
                if node.count_publishers('/cmd_vel') > 1:
                    raise RuntimeError('Another command publisher appeared')
                if not inside(camera_position(truth[-1])):
                    raise RuntimeError('Live camera reached conservative texture boundary')
                if stamps[-1] - truth[-1][0] > .2:
                    raise RuntimeError('Ground truth stream stale')
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = v, w
                pub.publish(cmd)
                rclpy.spin_once(node, timeout_sec=0.02)
                if time.monotonic() > limit:
                    raise RuntimeError('Simulation/image stalled')
    finally:
        for _ in range(10):
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.02)
        np.savez_compressed(out / 'live_capture.npz', frames=np.array(frames),
                            stamps=stamps, truth=np.array(truth),
                            wheel=np.array(wheel), motion=np.array(motion),
                            k=calibration[-1])
        (out / 'phases.json').write_text(json.dumps(phases, indent=2))
        print('Saved', len(frames), 'frames;', len(truth), 'truth poses', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
