"""Verify the actual ROS callback publishes the selected pose and twist axes."""
import math
from types import SimpleNamespace

from geometry_msgs.msg import Vector3Stamped
from optical_flow_odometry.metric_odometry_node import MetricOdometryNode
import pytest


class Recorder:
    """Capture messages without starting or altering live ROS nodes."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.mark.parametrize('reference_x,frame', [
    (0., 'body_link'), (-.15, 'drive_axle')])
def test_callback_reference_and_twist(reference_x, frame):
    theta, yaw, dt = .1, .4, .1
    # A body-origin turn gives camera displacement (R(theta)-I)r.
    # Compute its scene translation independently by inverse projection.
    c, s = math.cos(theta), math.sin(theta)
    camera_dx, camera_dy = (c - 1) * -.15, s * -.15
    gx = -c * camera_dx - s * camera_dy
    gy = s * camera_dx - c * camera_dy
    msg = Vector3Stamped()
    msg.header.stamp.sec = 1
    msg.header.stamp.nanosec = 100_000_000
    msg.vector.x = -gy * 160 / .085
    msg.vector.y = -gx * 160 / .085
    msg.vector.z = theta
    node = SimpleNamespace(
        fx=160., fy=160., h=.085, rx=-.15, ry=0.,
        reference_x=reference_x, reference_y=0., reference_frame=frame,
        previous_stamp=1_000_000_000, x=.2, y=.3, yaw=yaw, counter=0,
        delta_pub=Recorder(), odom_pub=Recorder())
    MetricOdometryNode.motion_callback(node, msg)
    # The selected point, attached at reference_x, moves (R-I)reference.
    dx, dy = (c - 1) * reference_x, s * reference_x
    assert node.x == pytest.approx(.2 + math.cos(yaw)*dx - math.sin(yaw)*dy)
    assert node.y == pytest.approx(.3 + math.sin(yaw)*dx + math.cos(yaw)*dy)
    assert node.yaw == pytest.approx(yaw + theta)
    delta = node.delta_pub.messages[-1]
    odom = node.odom_pub.messages[-1]
    assert delta.header.frame_id == odom.child_frame_id == frame
    assert delta.vector.x == pytest.approx(dx)
    assert delta.vector.y == pytest.approx(dy)
    assert odom.twist.twist.linear.x == pytest.approx((c*dx + s*dy) / dt)
    assert odom.twist.twist.linear.y == pytest.approx((-s*dx + c*dy) / dt)
    assert odom.twist.twist.angular.z == pytest.approx(theta / dt)
