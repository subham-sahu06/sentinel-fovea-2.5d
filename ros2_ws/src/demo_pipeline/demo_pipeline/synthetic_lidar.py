import math
import struct
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class SyntheticLidar(Node):
    """
    Tactical High-Density Synthetic LiDAR (15k+ points) with:
    - Multi-ring ground terrain & road curbs
    - Negative obstacles (military trenches & potholes)
    - Static obstacles (blast walls, perimeter trees, poles)
    - Dynamic moving targets (patrol vehicle, crossing pedestrian)
    - Overhanging clearance canopy
    - Kinematic odometry integration (/cmd_vel -> /odom)
    """

    def __init__(self) -> None:
        super().__init__('synthetic_lidar')
        self.publisher = self.create_publisher(PointCloud2, '/lidar/points', 10)
        self.odometry_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self.phase = 0.0
        self.cmd_vel = Twist()
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0

        self.create_timer(0.05, self._update_odometry)  # 20 Hz
        self.create_timer(0.10, self._publish)          # 10 Hz

    def _on_cmd_vel(self, message: Twist) -> None:
        self.cmd_vel = message

    def _update_odometry(self) -> None:
        dt = 0.05
        linear = float(self.cmd_vel.linear.x)
        angular = float(self.cmd_vel.angular.z)

        self.pose_x += linear * math.cos(self.yaw) * dt
        self.pose_y += linear * math.sin(self.yaw) * dt
        self.yaw += angular * dt

        odometry = Odometry()
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'
        odometry.pose.pose.position.x = self.pose_x
        odometry.pose.pose.position.y = self.pose_y
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odometry.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odometry.twist.twist.linear.x = linear
        odometry.twist.twist.angular.z = angular
        self.odometry_publisher.publish(odometry)

    def _publish(self) -> None:
        points_list = []
        t = self.phase

        # 1. Multi-Ring Ground Plane (32 rings, 360 azimuth rays = ~11,520 ground points)
        num_rings = 32
        rays_per_ring = 360
        radii = np.linspace(0.8, 60.0, num_rings)
        angles = np.linspace(0, 2 * np.pi, rays_per_ring, endpoint=False)

        # Vectorized ring computation
        r_grid, theta_grid = np.meshgrid(radii, angles)
        x_ground = (r_grid * np.cos(theta_grid)).flatten()
        y_ground = (r_grid * np.sin(theta_grid)).flatten()
        z_ground = np.full_like(x_ground, 0.02)
        intensity_ground = np.full_like(x_ground, 30.0)

        # 2. Road Curbs at y = ±3.5m
        curb_mask = (np.abs(y_ground) >= 3.3) & (np.abs(y_ground) <= 3.7) & (x_ground > -5.0) & (x_ground < 40.0)
        z_ground[curb_mask] = 0.18
        intensity_ground[curb_mask] = 75.0

        # 3. Negative Obstacle 1: Military Trench at x in [5.5, 6.8], y in [-3.0, 3.0]
        trench_mask = (x_ground >= 5.5) & (x_ground <= 6.8) & (np.abs(y_ground) <= 3.0)
        z_ground[trench_mask] = -0.45
        intensity_ground[trench_mask] = 10.0

        # 4. Negative Obstacle 2: Road Pothole at x ~ 3.2m, y ~ 1.2m
        pothole_dist = np.hypot(x_ground - 3.2, y_ground - 1.2)
        pothole_mask = pothole_dist <= 0.6
        z_ground[pothole_mask] = -0.22

        # Combine ground points
        ground_points = np.stack([x_ground, y_ground, z_ground, intensity_ground], axis=-1)
        points_list.append(ground_points)

        # 5. Static Obstacles (Blast Wall, Perimeter Poles, Barrier Blocks)
        static_points = []
        # Concrete Blast Wall at (14m, -4.5m)
        for wx in np.linspace(12.0, 16.0, 25):
            for wz in np.linspace(0.0, 1.4, 15):
                static_points.append([wx, -4.5, wz, 90.0])

        # Security Perimeter Poles at x = 22m, 45m, 65m
        for px, py in [(22.0, -8.0), (22.0, 8.0), (45.0, -12.0), (45.0, 12.0), (65.0, -15.0), (65.0, 15.0)]:
            for pz in np.linspace(0.0, 3.5, 20):
                static_points.append([px, py, pz, 120.0])

        if static_points:
            points_list.append(np.array(static_points, dtype=np.float32))

        # 6. Dynamic Obstacle 1: Moving Patrol Vehicle at x(t) in [16m, 32m], y = 2.0m
        veh_x = 22.0 + 7.0 * math.sin(0.4 * t)
        veh_y = 2.0
        veh_points = []
        for vx in np.linspace(-1.8, 1.8, 16):
            for vy in np.linspace(-0.9, 0.9, 10):
                for vz in np.linspace(0.2, 1.5, 8):
                    veh_points.append([veh_x + vx, veh_y + vy, vz, 180.0])
        points_list.append(np.array(veh_points, dtype=np.float32))

        # 7. Dynamic Obstacle 2: Crossing Dismounted Personnel / Pedestrian at x = 8.5m
        ped_x = 8.5
        ped_y = 2.8 * math.sin(0.6 * t)
        ped_points = []
        for px in np.linspace(-0.25, 0.25, 6):
            for py in np.linspace(-0.25, 0.25, 6):
                for pz in np.linspace(0.0, 1.75, 14):
                    ped_points.append([ped_x + px, ped_y + py, pz, 220.0])
        points_list.append(np.array(ped_points, dtype=np.float32))

        # 8. Overhanging Clearance Canopy at x = 18.0m, z = 2.2m
        canopy_points = []
        for cx in np.linspace(17.5, 18.5, 8):
            for cy in np.linspace(-4.0, 4.0, 24):
                canopy_points.append([cx, cy, 2.2, 85.0])
        points_list.append(np.array(canopy_points, dtype=np.float32))

        # Concatenate all point arrays (~14,000 - 18,000 points)
        all_points = np.concatenate(points_list, axis=0).astype(np.float32)

        # Build PointCloud2 Message
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.height = 1
        message.width = int(all_points.shape[0])
        message.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 16
        message.row_step = message.point_step * message.width
        message.data = all_points.tobytes()

        self.publisher.publish(message)
        self.phase += 0.08


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticLidar()
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