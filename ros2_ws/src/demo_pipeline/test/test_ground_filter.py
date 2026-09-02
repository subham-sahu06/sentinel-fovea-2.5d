import struct
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from demo_pipeline.ground_filter import GroundFilter


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def create_test_cloud(points):
    msg = PointCloud2()
    msg.header.frame_id = 'base_link'
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.data = b''.join(struct.pack('<fff', *p) for p in points)
    return msg


def test_ground_filter_filtering(rclpy_init):
    node = GroundFilter()
    try:
        received = []
        node.publisher = type('MockPub', (), {'publish': staticmethod(lambda m: received.append(m))})()

        # Cloud with 2 ground points (z <= 0.08) and 2 obstacle points (z > 0.08)
        points = [
            (1.0, 2.0, 0.02),  # Ground -> remove
            (2.0, 1.0, 0.05),  # Ground -> remove
            (3.0, 0.0, 0.50),  # Obstacle -> keep
            (0.0, 4.0, 0.80),  # Obstacle -> keep
        ]
        input_cloud = create_test_cloud(points)
        node._filter(input_cloud)

        assert len(received) == 1
        output = received[0]
        assert output.width == 2
        assert output.height == 1

        # Unpack output points
        out_data = bytes(output.data)
        p1 = struct.unpack_from('<fff', out_data, 0)
        p2 = struct.unpack_from('<fff', out_data, 12)
        assert abs(p1[2] - 0.50) < 1e-5
        assert abs(p2[2] - 0.80) < 1e-5
    finally:
        node.destroy_node()


def test_ground_filter_malformed_input(rclpy_init):
    node = GroundFilter()
    try:
        received = []
        node.publisher = type('MockPub', (), {'publish': staticmethod(lambda m: received.append(m))})()

        # Empty cloud
        empty_cloud = PointCloud2()
        node._filter(empty_cloud)
        assert len(received) == 0

        # Bad point step
        bad_cloud = create_test_cloud([(1.0, 1.0, 1.0)])
        bad_cloud.point_step = 0
        node._filter(bad_cloud)
        assert len(received) == 0
    finally:
        node.destroy_node()

