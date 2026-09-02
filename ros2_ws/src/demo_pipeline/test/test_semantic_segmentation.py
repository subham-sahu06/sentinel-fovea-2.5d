import struct
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from demo_pipeline.semantic_segmentation import SemanticSegmentation


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def create_cloud(points):
    msg = PointCloud2()
    msg.header.frame_id = 'base_link'
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.data = b''.join(struct.pack('<ffff', *p) for p in points)
    return msg


def test_semantic_segmentation_classification(rclpy_init):
    node = SemanticSegmentation()
    try:
        received = []
        node.publisher = type('MockPub', (), {'publish': staticmethod(lambda m: received.append(m))})()

        # Test points:
        # 1. Flat ground -> Drivable (Class 0)
        # 2. Trench point (z = -0.35) -> Negative/Trench (Class 1)
        # 3. Wall obstacle (x=14, y=-4.5, z=0.8) -> Static (Class 2)
        # 4. Dynamic crossing pedestrian (x=8.5, y=0.5, z=1.0) -> Dynamic (Class 3)
        points = [
            (2.0, 0.0, 0.0, 40.0),        # Drivable
            (6.0, 0.0, -0.35, 10.0),      # Trench
            (14.0, -4.5, 0.8, 90.0),      # Wall
            (8.5, 0.5, 1.0, 200.0),       # Dynamic ped
        ]
        cloud = create_cloud(points)
        node._on_cloud(cloud)

        assert len(received) == 1
        output = received[0]
        assert output.width == 4

        # Read classes from output (point_step = 24, class_id offset = 16)
        out_bytes = bytes(output.data)
        classes = []
        for i in range(4):
            offset = i * 24
            cid = struct.unpack_from('<f', out_bytes, offset + 16)[0]
            classes.append(int(cid))

        assert classes[0] == 0  # Drivable
        assert classes[1] == 1  # Negative/Trench
        assert classes[2] == 2  # Static Obstacle
        assert classes[3] == 3  # Dynamic Target
    finally:
        node.destroy_node()

