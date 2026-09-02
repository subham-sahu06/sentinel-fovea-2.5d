import json
import struct
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from adaptive_grid.node import AdaptiveGrid, FoveaCell


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def create_semantic_cloud(points):
    # points: tuple of (x, y, z, intensity, class_id, confidence)
    msg = PointCloud2()
    msg.header.frame_id = 'base_link'
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='class_id', offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name='confidence', offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 24
    msg.row_step = msg.point_step * msg.width
    msg.data = b''.join(struct.pack('<ffffff', *p) for p in points)
    return msg


def test_foveated_grid_init(rclpy_init):
    node = AdaptiveGrid()
    try:
        assert node.z1_radius == 10.0
        assert node.z1_res == 0.05
        assert node.z2_radius == 30.0
        assert node.z2_res == 0.15
        assert node.z3_radius == 100.0
        assert node.z3_res == 0.50
    finally:
        node.destroy_node()


def test_foveated_grid_point_processing(rclpy_init):
    node = AdaptiveGrid()
    try:
        occupancy_msgs = []
        marker_msgs = []
        metric_msgs = []
        node.occupancy_publisher = type('MockPub', (), {'publish': staticmethod(lambda m: occupancy_msgs.append(m))})()
        node.marker_publisher = type('MockPub', (), {'publish': staticmethod(lambda m: marker_msgs.append(m))})()
        node.metrics_publisher = type('MockPub', (), {'publish': staticmethod(lambda m: metric_msgs.append(m))})()

        # Points in 3 different zones:
        # Zone 1 (<10m): Trench at (6m, 0m, -0.35m) -> Class 1
        # Zone 2 (10-30m): Static Wall at (14m, -4.5m, 0.8m) -> Class 2
        # Zone 3 (30-100m): Perimeter tree at (45m, 12m, 1.5m) -> Class 2
        # Near Drivable: (2m, 0m, 0.0m) -> Class 0
        points = [
            (2.0, 0.0, 0.0, 40.0, 0.0, 0.95),       # Zone 1 Drivable (5cm res)
            (6.0, 0.0, -0.35, 10.0, 1.0, 0.92),     # Zone 1 Trench (5cm res)
            (14.0, -4.5, 0.8, 90.0, 2.0, 0.88),     # Zone 2 Wall (15cm res)
            (45.0, 12.0, 1.5, 120.0, 2.0, 0.85),    # Zone 3 Tree (50cm res)
        ]
        cloud = create_semantic_cloud(points)
        node._on_cloud(cloud)

        assert len(occupancy_msgs) == 1
        assert len(marker_msgs) == 1
        assert len(metric_msgs) == 1

        # Check metrics payload
        metrics = json.loads(metric_msgs[0].data)
        assert metrics['uniform_5cm_memory_mb'] == 64.0
        assert metrics['memory_savings_percent'] > 90.0
        assert metrics['active_cells_count'] == 4

        # Markers
        markers = marker_msgs[0].markers
        assert len(markers) == 4
    finally:
        node.destroy_node()
