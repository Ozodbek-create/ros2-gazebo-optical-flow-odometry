"""Convert floor-image transforms to a selected robot reference pose."""

import json
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from optical_flow_odometry.motion_geometry import (
    image_to_reference, integrate_pose)
from optical_flow_odometry.tracking import IntervalGuard
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String


class MetricOdometryNode(Node):

    def __init__(self):
        super().__init__('metric_odometry_node')

        # Camera mounting relative to body_link. The axle is at x=-0.15.
        # Keep the physical mounting even when selecting axle odometry.
        self.declare_parameter('camera_height', 0.085)
        self.declare_parameter('camera_x', -0.15)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('reference_x', 0.0)
        self.declare_parameter('reference_y', 0.0)
        self.declare_parameter('reference_frame', 'body_link')
        self.reference_x = float(self.get_parameter('reference_x').value)
        self.reference_y = float(self.get_parameter('reference_y').value)
        self.reference_frame = str(self.get_parameter('reference_frame').value)
        if (self.reference_x or self.reference_y) and self.reference_frame == 'body_link':
            raise ValueError('An offset reference requires a distinct reference_frame')

        self.h = float(
            self.get_parameter('camera_height').value
        )
        self.rx = float(
            self.get_parameter('camera_x').value
        )
        self.ry = float(
            self.get_parameter('camera_y').value
        )

        if not all(map(math.isfinite, (
                self.h, self.rx, self.ry, self.reference_x, self.reference_y))) or self.h <= 0:
            raise ValueError('Camera/reference geometry must be finite; height > 0')

        self.fx = None
        self.fy = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.guard = IntervalGuard()
        self.interval_dt = None
        self.last_arrival = time.monotonic()
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(String, '/optical_flow/measurement',
                                 self.measurement_callback, 10)
        self.create_timer(1., self.watchdog)
        self.declare_parameter('legacy_input', False)
        self.previous_stamp = None
        self.counter = 0
        self.warned_camera = False

        self.create_subscription(
            CameraInfo,
            '/camera/down/camera_info',
            self.camera_callback,
            qos_profile_sensor_data
        )

        if self.get_parameter('legacy_input').value:
            self.create_subscription(Vector3Stamped, '/optical_flow/image_motion',
                                     self.motion_callback, 10)
            self.get_logger().warn('Legacy input has no interval or quality guarantees')

        self.odom_pub = self.create_publisher(
            Odometry,
            '/optical_flow/odom',
            10
        )

        self.delta_pub = self.create_publisher(
            Vector3Stamped,
            '/optical_flow/delta_body',
            10
        )

        self.get_logger().info(
            'Metric optical odometry started'
        )

    def camera_callback(self, msg):

        fx = float(msg.k[0])
        fy = float(msg.k[4])

        if math.isfinite(fx) and math.isfinite(fy) and fx > 0 and fy > 0:
            if self.fx is None:
                self.get_logger().info(
                    f'Camera calibrated: fx={fx:.2f}, '
                    f'fy={fy:.2f}'
                )
            self.fx = fx
            self.fy = fy

    def diagnose(self, reason, level):
        """Report admission counts and latched pose incompleteness."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='optical_flow/odometry', hardware_id='down_camera',
                                  level=level, message=reason)
        status.values = [KeyValue(key='pose_incomplete', value=str(self.guard.incomplete)),
                         KeyValue(key='accepted', value=str(self.guard.accepted)),
                         KeyValue(key='rejected', value=str(self.guard.rejected))]
        msg.status = [status]
        self.diagnostic_pub.publish(msg)

    def watchdog(self):
        """Report lost measurement input even without new callbacks."""
        if time.monotonic() - self.last_arrival > 1.:
            self.diagnose('no_recent_measurements', DiagnosticStatus.STALE)

    def measurement_callback(self, msg):
        """Validate atomic motion and use its own measurement interval."""
        if self.get_parameter('legacy_input').value:
            return
        self.last_arrival = time.monotonic()
        try:
            m = json.loads(msg.data)
            if not isinstance(m, dict):
                raise ValueError('record must be an object')
            accepted, reason = self.guard.admit(m)
        except (ValueError, TypeError):
            self.guard.incomplete = True
            self.guard.rejected += 1
            accepted, reason = False, 'malformed_json'
        if accepted and (self.fx is None or self.fy is None):
            self.guard.incomplete = True
            accepted, reason = False, 'no_metric_calibration'
        if accepted:
            motion = Vector3Stamped()
            motion.header.stamp.sec, motion.header.stamp.nanosec = divmod(m['end_ns'], 10**9)
            motion.vector.x, motion.vector.y, motion.vector.z = m['du'], m['dv'], m['dtheta']
            self.interval_dt = (m['end_ns'] - m['start_ns']) / 1e9
            try:
                self.motion_callback(motion)
            finally:
                self.interval_dt = None
        level = DiagnosticStatus.ERROR if self.guard.incomplete else (
            DiagnosticStatus.OK if accepted else DiagnosticStatus.WARN)
        self.diagnose(reason, level)

    def motion_callback(self, msg):

        if hasattr(self, 'guard') and self.interval_dt is None:
            self.last_arrival = time.monotonic()
        if self.fx is None or self.fy is None:
            if not self.warned_camera:
                self.get_logger().warn(
                    'Waiting for CameraInfo'
                )
                self.warned_camera = True
            return

        stamp = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )

        if getattr(self, 'interval_dt', None) is not None:
            dt = self.interval_dt
        else:
            # Compatibility only: a Vector3Stamped cannot convey its start.
            if self.previous_stamp is None:
                self.previous_stamp = stamp
                return
            dt = (stamp - self.previous_stamp) / 1e9
            if dt <= 0:
                return
            self.previous_stamp = stamp
            if dt > .2:
                return

        du = float(msg.vector.x)
        dv = float(msg.vector.y)
        dtheta = float(msg.vector.z)

        if not all(map(math.isfinite, (du, dv, dtheta))):
            return

        # Initial sanity limits against bad estimates.
        if abs(du) > 50 or abs(dv) > 50:
            return

        if abs(dtheta) > 0.25:
            return

        body_dx, body_dy = image_to_reference(
            du, dv, dtheta, self.h, self.fx, self.fy, self.rx, self.ry,
            self.reference_x, self.reference_y)
        self.x, self.y, self.yaw = integrate_pose(
            self.x, self.y, self.yaw, body_dx, body_dy, dtheta)

        # Publish incremental body motion in SI units.
        delta = Vector3Stamped()
        delta.header = msg.header
        delta.header.frame_id = self.reference_frame
        delta.vector.x = body_dx
        delta.vector.y = body_dy
        delta.vector.z = dtheta
        self.delta_pub.publish(delta)

        # Publish accumulated odometry.
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'optical_odom'
        odom.child_frame_id = self.reference_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.z = math.sin(
            self.yaw / 2.0
        )
        odom.pose.pose.orientation.w = math.cos(
            self.yaw / 2.0
        )

        # Odometry twist uses CURRENT child axes, while delta uses the
        # previous child axes. This is the interval-average displacement.
        c, s = math.cos(dtheta), math.sin(dtheta)
        odom.twist.twist.linear.x = (c * body_dx + s * body_dy) / dt
        odom.twist.twist.linear.y = (-s * body_dx + c * body_dy) / dt
        odom.twist.twist.angular.z = dtheta / dt

        # Preliminary conservative uncertainty values.
        # Replace with experimentally calibrated values.
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36

        pose_cov[0] = 0.01
        pose_cov[7] = 0.01
        pose_cov[14] = 1e6
        pose_cov[21] = 1e6
        pose_cov[28] = 1e6
        pose_cov[35] = 0.03

        twist_cov[0] = 0.04
        twist_cov[7] = 0.04
        twist_cov[14] = 1e6
        twist_cov[21] = 1e6
        twist_cov[28] = 1e6
        twist_cov[35] = 0.25

        if getattr(self, 'guard', None) is not None and self.guard.incomplete:
            for axis in (0, 7, 35):
                pose_cov[axis] = 1e6
        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        self.counter += 1

        if self.counter % 30 == 0:
            self.get_logger().info(
                f'x={self.x:+.3f}m '
                f'y={self.y:+.3f}m '
                f'yaw={math.degrees(self.yaw):+.1f}deg'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MetricOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
