#!/usr/bin/env python3
"""
Real LiDAR Dataset Player Node

Reads authentic point cloud data from CSV and publishes to /lidar/points topic.
Cycles through frames at configurable rate for testing semantic segmentation pipeline.

Features:
- Reads from dataset.csv (12,000 balanced LiDAR points)
- 4 tactical defense classes with authentic LiDAR characteristics
- Configurable playback rate (default: 10 Hz)
- Loops dataset for continuous operation
"""

import math
import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import os
from pathlib import Path


class DatasetPlayer(Node):
    """
    Publishes real LiDAR points from trained dataset to /lidar/points topic.
    Integrates with odometry for kinematic simulation.
    """

    def __init__(self) -> None:
        super().__init__('dataset_player')
        
        # Publisher for LiDAR points
        self.publisher = self.create_publisher(PointCloud2, '/lidar/points', 10)
        
        # Publisher for odometry feedback
        self.odometry_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Subscriber for velocity commands
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        
        # Load dataset
        self.dataset_df = None
        self.frames_per_scan = 1500  # Sample this many points per "scan" at 10 Hz
        
        self._load_dataset()
        
        # Odometry state
        self.cmd_vel = Twist()
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        
        # Timers
        self.create_timer(0.05, self._update_odometry)   # 20 Hz odometry
        self.create_timer(0.10, self._publish_frame)     # 10 Hz LiDAR
        
        self.get_logger().info(f'Dataset Player initialized with {len(self.dataset_df)} points')
        self.get_logger().info(f'Publishing frames of {self.frames_per_scan} points at 10 Hz (proportional class sampling)')

    def _load_dataset(self) -> None:
        """Load point cloud data from dataset.csv"""
        # Try multiple possible paths
        possible_paths = [
            '/home/subham/robot-dashboard/dataset.csv',
            'dataset.csv',
            '../../../dataset.csv',
            str(Path.home() / 'robot-dashboard' / 'dataset.csv'),
        ]
        
        dataset_path = None
        for path in possible_paths:
            if os.path.exists(path):
                dataset_path = path
                break
        
        if dataset_path is None:
            self.get_logger().error('Dataset file not found in any of the following locations:')
            for path in possible_paths:
                self.get_logger().error(f'  - {path}')
            raise FileNotFoundError('dataset.csv not found')
        
        self.get_logger().info(f'Loading dataset from: {dataset_path}')
        self.dataset_df = pd.read_csv(dataset_path)
        
        # Verify required columns
        required_cols = ['x', 'y', 'z', 'intensity', 'semantic_class_id', 'semantic_class_name']
        missing_cols = [col for col in required_cols if col not in self.dataset_df.columns]
        if missing_cols:
            self.get_logger().error(f'Dataset missing required columns: {missing_cols}')
            raise ValueError(f'Missing columns: {missing_cols}')
        
        # Log class distribution
        self.get_logger().info('Dataset class distribution:')
        for class_id in sorted(self.dataset_df['semantic_class_id'].unique()):
            class_name = self.dataset_df[self.dataset_df['semantic_class_id'] == class_id]['semantic_class_name'].iloc[0]
            count = len(self.dataset_df[self.dataset_df['semantic_class_id'] == class_id])
            self.get_logger().info(f'  Class {class_id} ({class_name}): {count} points')

    def _on_cmd_vel(self, message: Twist) -> None:
        """Receive velocity commands"""
        self.cmd_vel = message

    def _update_odometry(self) -> None:
        """Update pose based on velocity commands"""
        dt = 0.05
        linear = float(self.cmd_vel.linear.x)
        angular = float(self.cmd_vel.angular.z)

        # Integrate kinematics
        self.pose_x += linear * math.cos(self.yaw) * dt
        self.pose_y += linear * math.sin(self.yaw) * dt
        self.yaw += angular * dt

        # Publish odometry
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

    def _publish_frame(self) -> None:
        """Publish a frame of LiDAR points sampled proportionally from all classes"""
        if self.dataset_df is None or len(self.dataset_df) == 0:
            return
        
        # Sample proportionally from each semantic class
        frame_data = self._sample_proportional_frame()
        
        # Extract point cloud data
        x = frame_data['x'].values.astype(np.float32)
        y = frame_data['y'].values.astype(np.float32)
        z = frame_data['z'].values.astype(np.float32)
        intensity = frame_data['intensity'].values.astype(np.float32)
        
        # Create PointCloud2 message
        cloud_msg = self._create_pointcloud2(x, y, z, intensity)
        self.publisher.publish(cloud_msg)

    def _sample_proportional_frame(self) -> pd.DataFrame:
        """
        Sample points proportionally from each semantic class.
        Ensures all 4 tactical classes (DRIVABLE, NEGATIVE_TRENCH, STATIC_OBSTACLE, DYNAMIC_TARGET)
        are represented equally in each frame for balanced visualization.
        """
        frame_data = []
        
        # Get unique classes
        classes = sorted(self.dataset_df['semantic_class_id'].unique())
        
        # Calculate points per class (equal distribution)
        points_per_class = self.frames_per_scan // len(classes)
        remainder = self.frames_per_scan % len(classes)
        
        for i, class_id in enumerate(classes):
            class_df = self.dataset_df[self.dataset_df['semantic_class_id'] == class_id]
            
            # Add one extra point to first few classes to handle remainder
            num_points = points_per_class + (1 if i < remainder else 0)
            
            # Sample with replacement to ensure we always get enough points
            sampled = class_df.sample(n=num_points, replace=True, random_state=None)
            frame_data.append(sampled)
        
        # Concatenate all classes and shuffle for random point order
        frame_df = pd.concat(frame_data, ignore_index=True)
        return frame_df.sample(frac=1).reset_index(drop=True)

    def _create_pointcloud2(self, x: np.ndarray, y: np.ndarray,
                            z: np.ndarray, intensity: np.ndarray) -> PointCloud2:
        """Create a PointCloud2 message from numpy arrays"""
        num_points = len(x)
        
        # Create PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header.frame_id = 'lidar'
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.height = 1
        cloud_msg.width = num_points
        cloud_msg.is_dense = True
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16  # 4 fields × 4 bytes each
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        
        # Define fields: x, y, z, intensity (all float32)
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Pack data into binary format
        points_data = np.column_stack([x, y, z, intensity]).astype(np.float32)
        cloud_msg.data = points_data.tobytes()
        
        return cloud_msg


def main(args=None):
    rclpy.init(args=args)
    node = DatasetPlayer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
