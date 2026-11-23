"""Publish optical motion, explicit interval records, and ROS diagnostics."""
import json
import time

import cv2
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
import numpy as np
from optical_flow_odometry.tracking import Tracker
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class OpticalFlowNode(Node):
    """Keep the legacy vector topic while exposing failures on every image."""

    def __init__(self):
        super().__init__('optical_flow_node')
        self.bridge = CvBridge()
        self.intrinsics = None
        self.tracker = Tracker()
        self.last_arrival = time.monotonic()
        self.failures = 0
        self.motion_pub = self.create_publisher(Vector3Stamped, '/optical_flow/image_motion', 10)
        self.measurement_pub = self.create_publisher(String, '/optical_flow/measurement', 10)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.debug_pub = self.create_publisher(Image, '/optical_flow/debug_image', 10)
        self.create_subscription(CameraInfo, '/camera/down/camera_info', self.camera_callback,
                                 qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image, '/camera/down/image_raw', self.image_callback, qos_profile_sensor_data)
        self.create_timer(1., self.watchdog)

    def camera_callback(self, msg):
        """Accept only finite positive focal calibration."""
        k = np.asarray(msg.k)
        if np.isfinite(k).all() and k[0] > 0 and k[4] > 0:
            new = (k[0], k[4], k[2], k[5])
            if self.intrinsics is not None and new != self.intrinsics:
                self.tracker.previous = None
            self.intrinsics = new

    def diagnose(self, fields, level, reason, stamp=None):
        """Expose raw metrics rather than an uncalibrated confidence score."""
        msg = DiagnosticArray()
        msg.header.stamp = stamp or self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='optical_flow/tracking', hardware_id='down_camera',
                                  level=level, message=reason)
        status.values = [KeyValue(key=k, value=str(v)) for k, v in fields.items()]
        msg.status = [status]
        self.diagnostic_pub.publish(msg)

    def watchdog(self):
        """Report input loss even when image callbacks have stopped."""
        age = time.monotonic() - self.last_arrival
        if age > 1.:
            self.diagnose({'wall_age_s': age}, DiagnosticStatus.STALE, 'no_recent_images')

    def image_callback(self, msg):
        """Process one frame and publish its atomic measurement record."""
        started = self.last_arrival = time.monotonic()
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            m = self.tracker.process(gray, stamp, self.intrinsics)
        except Exception as exc:
            self.diagnose({'error': str(exc)}, DiagnosticStatus.ERROR, 'conversion_failed',
                          msg.header.stamp)
            return
        m['frame_id'] = msg.header.frame_id
        m['processing_ms'] = (time.monotonic() - started) * 1000
        self.measurement_pub.publish(String(data=json.dumps(m, allow_nan=False)))
        if m['valid']:
            motion = Vector3Stamped(header=msg.header)
            motion.vector.x, motion.vector.y, motion.vector.z = m['du'], m['dv'], m['dtheta']
            self.motion_pub.publish(motion)
        else:
            self.failures += 1
        self.diagnose({**m, 'failures_total': self.failures},
                      DiagnosticStatus.OK if m['valid'] else DiagnosticStatus.WARN,
                      m['reason'], msg.header.stamp)
        visual = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.putText(visual, f"{m['reason']} inliers={m['inliers']}/{m['tracked']}",
                    (5, 20), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
        debug = self.bridge.cv2_to_imgmsg(visual, encoding='bgr8')
        debug.header = msg.header
        self.debug_pub.publish(debug)


def main(args=None):
    """Run the ROS image node."""
    rclpy.init(args=args)
    node = OpticalFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
