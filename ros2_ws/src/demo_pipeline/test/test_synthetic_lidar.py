import math
import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from demo_pipeline.synthetic_lidar import SyntheticLidar


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_synthetic_lidar_init(rclpy_init):
    node = SyntheticLidar()
    try:
        assert node.pose_x == 0.0
        assert node.pose_y == 0.0
        assert node.yaw == 0.0
        assert node.cmd_vel.linear.x == 0.0
        assert node.cmd_vel.angular.z == 0.0
    finally:
        node.destroy_node()


def test_odometry_kinematics(rclpy_init):
    node = SyntheticLidar()
    try:
        # Straight line motion
        cmd = Twist()
        cmd.linear.x = 2.0
        cmd.angular.z = 0.0
        node._on_cmd_vel(cmd)

        node._update_odometry()
        assert abs(node.pose_x - (2.0 * 0.05)) < 1e-6
        assert abs(node.pose_y - 0.0) < 1e-6
        assert abs(node.yaw - 0.0) < 1e-6

        # Rotation
        cmd.linear.x = 0.0
        cmd.angular.z = 1.0
        node._on_cmd_vel(cmd)

        node._update_odometry()
        assert abs(node.yaw - 0.05) < 1e-6
    finally:
        node.destroy_node()


def test_synthetic_pointcloud_generation(rclpy_init):
    node = SyntheticLidar()
    try:
        received = []
        node.publisher = type('MockPub', (), {'publish': staticmethod(lambda m: received.append(m))})()
        node._publish()
        assert len(received) == 1
        msg = received[0]
        # Multi-ring point cloud should have over 10,000 points
        assert msg.width >= 10000
        assert msg.point_step == 16
    finally:
        node.destroy_node()

