#!/usr/bin/env python3
"""
ROS 2 Semantic Segmentation Node (Inference)
Uses ONNX model for real-time 3D semantic segmentation of LiDAR point clouds
"""

import os
import numpy as np
import onnxruntime as ort
import torch
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from ros2_lidar_types.msg import SemanticPointCloud
from std_msgs.msg import Header


class SemanticSegmentationNode(Node):
    """
    Performs real-time semantic segmentation on LiDAR point clouds.
    Reads: /lidar/points (PointCloud2)
    Publishes: /perception/semantic_cloud (SemanticPointCloud)
    """
    
    # Class ID to name mapping (must match training)
    CLASS_NAMES = {
        0: 'DRIVABLE',
        1: 'NEGATIVE_TRENCH',
        2: 'STATIC_OBSTACLE',
        3: 'DYNAMIC_TARGET'
    }
    
    def __init__(self):
        super().__init__('semantic_segmentation_node')
        
        # Parameters
        self.declare_parameter('model_path', './semantic_model.onnx')
        self.declare_parameter('num_points', 4096)
        self.declare_parameter('confidence_threshold', 0.5)
        
        model_path = self.get_parameter('model_path').value
        self.num_points = self.get_parameter('num_points').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        self.get_logger().info(f"Loading ONNX model from: {model_path}")
        
        # Load ONNX model
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        self.session = ort.InferenceSession(
            model_path,
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
        )
        self.get_logger().info("✓ ONNX model loaded")
        
        # Input/output names
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        
        # Subscribe and publish
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.pointcloud_callback,
            qos_profile=10
        )
        
        self.publisher = self.create_publisher(
            SemanticPointCloud,
            '/perception/semantic_cloud',
            10
        )
        
        self.get_logger().info("✓ Semantic segmentation node initialized")
    
    def pointcloud_callback(self, msg: PointCloud2) -> None:
        """Process incoming point cloud and publish predictions"""
        try:
            # Convert ROS PointCloud2 to numpy array
            points = self._pointcloud2_to_array(msg)
            
            if len(points) == 0:
                self.get_logger().warn("Empty point cloud received")
                return
            
            # Preprocess: sample/pad to fixed size
            points_normalized = self._preprocess_points(points)
            
            # Run inference
            logits = self.session.run(
                [self.output_name],
                {self.input_name: points_normalized}
            )[0]  # (1, num_points, 4)
            
            # Get predictions
            class_ids = np.argmax(logits[0], axis=1)  # (num_points,)
            confidences = np.max(logits[0], axis=1)  # (num_points,)
            
            # Create output message
            semantic_msg = SemanticPointCloud()
            semantic_msg.header = msg.header
            semantic_msg.points = msg.data
            semantic_msg.class_ids = class_ids.astype(np.int32).tobytes()
            semantic_msg.confidences = confidences.astype(np.float32).tobytes()
            
            self.publisher.publish(semantic_msg)
            
            self.get_logger().debug(
                f"Segmented {len(points)} points in {msg.header.frame_id}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")
    
    def _pointcloud2_to_array(self, msg: PointCloud2) -> np.ndarray:
        """Convert ROS PointCloud2 message to numpy array"""
        # Extract x, y, z, intensity from PointCloud2
        points = []
        offset = 0
        
        for point_idx in range(msg.width * msg.height):
            x_offset = offset + 0
            y_offset = offset + 4
            z_offset = offset + 8
            intensity_offset = offset + 16
            
            x = np.frombuffer(msg.data[x_offset:x_offset+4], dtype=np.float32)[0]
            y = np.frombuffer(msg.data[y_offset:y_offset+4], dtype=np.float32)[0]
            z = np.frombuffer(msg.data[z_offset:z_offset+4], dtype=np.float32)[0]
            intensity = np.frombuffer(msg.data[intensity_offset:intensity_offset+4], dtype=np.float32)[0]
            
            points.append([x, y, z, intensity])
            offset += msg.point_step
        
        return np.array(points, dtype=np.float32)
    
    def _preprocess_points(self, points: np.ndarray) -> np.ndarray:
        """Normalize and pad/sample points to fixed size"""
        n_pts = len(points)
        
        if n_pts >= self.num_points:
            # Downsampling
            indices = np.random.choice(n_pts, self.num_points, replace=False)
            points = points[indices]
        else:
            # Upsampling
            repeat_count = self.num_points // n_pts
            remainder = self.num_points % n_pts
            points = np.vstack([points] * repeat_count)
            if remainder > 0:
                idx_remainder = np.random.choice(n_pts, remainder, replace=False)
                points = np.vstack([points, points[idx_remainder]])
        
        # Normalize
        points = points.astype(np.float32)
        points -= np.mean(points, axis=0)
        points /= np.std(points, axis=0) + 1e-8
        
        # Add batch dimension
        return points[np.newaxis, :, :]  # (1, num_points, 4)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticSegmentationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
