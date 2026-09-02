import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class GroundFilter(Node):
    def __init__(self) -> None:
        super().__init__('ground_filter')
        self.publisher = self.create_publisher(PointCloud2, '/filtered_points', 10)
        self.create_subscription(PointCloud2, '/lidar/points', self._filter, 10)

    def _filter(self, message: PointCloud2) -> None:
        fields = {field.name: field for field in message.fields}
        if any(name not in fields for name in ('x', 'y', 'z')) or message.point_step <= 0:
            return
        data = bytes(message.data)
        count = min(message.width * max(message.height, 1), len(data) // message.point_step)
        filtered = []
        for index in range(count):
            offset = index * message.point_step
            x, y, z = (struct.unpack_from('<f', data, offset + fields[name].offset)[0] for name in ('x', 'y', 'z'))
            if all(math.isfinite(value) for value in (x, y, z)) and z > 0.08:
                filtered.append((x, y, z))
        output = PointCloud2()
        output.header = message.header
        output.height = 1
        output.width = len(filtered)
        output.fields = [PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1) for index, name in enumerate(('x', 'y', 'z'))]
        output.is_bigendian = False
        output.point_step = 12
        output.row_step = output.point_step * output.width
        output.data = b''.join(struct.pack('<fff', *point) for point in filtered)
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()