#!/usr/bin/env python3
"""
Test script for ONNX semantic segmentation inference
Validates model loading, inference, and output format
"""

import os
import sys
import argparse
import numpy as np

try:
    import onnx
    import onnxruntime as ort
except ImportError:
    print("ERROR: onnxruntime not installed. Run: pip install onnxruntime")
    sys.exit(1)


class SemanticSegmentationInferencer:
    """Lightweight ONNX inference wrapper for semantic segmentation"""
    
    CLASS_NAMES = {
        0: 'DRIVABLE',
        1: 'NEGATIVE_TRENCH',
        2: 'STATIC_OBSTACLE',
        3: 'DYNAMIC_TARGET'
    }
    
    def __init__(self, model_path, num_points=4096):
        """Load ONNX model"""
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        print(f"Loading model from: {model_path}")
        self.session = ort.InferenceSession(
            model_path,
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
        )
        
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.num_points = num_points
        
        print(f"✓ Model loaded successfully")
        print(f"  Input: {self.input_name}")
        print(f"  Output: {self.output_name}")
        print(f"  Execution providers: {self.session.get_providers()}")
    
    def infer(self, points):
        """
        Run inference on point cloud
        Args:
            points: (num_points, 4) array of x, y, z, intensity
        Returns:
            class_ids: (num_points,) predicted class IDs
            confidences: (num_points,) confidence scores
        """
        # Preprocess
        if len(points) != self.num_points:
            print(f"Warning: Expected {self.num_points} points, got {len(points)}")
            points = self._pad_or_sample(points)
        
        # Add batch dimension
        input_data = points[np.newaxis, :, :].astype(np.float32)
        
        # Inference
        logits = self.session.run(
            [self.output_name],
            {self.input_name: input_data}
        )[0]  # (1, num_points, 4)
        
        # Extract predictions
        class_ids = np.argmax(logits[0], axis=1)
        confidences = np.max(logits[0], axis=1)
        
        return class_ids, confidences
    
    def _pad_or_sample(self, points):
        """Pad or downsample to fixed size"""
        n = len(points)
        if n >= self.num_points:
            idx = np.random.choice(n, self.num_points, replace=False)
            return points[idx]
        else:
            idx = np.random.choice(n, self.num_points, replace=True)
            return points[idx]


def test_inference(model_path, num_test_samples=5):
    """Test ONNX model with synthetic point clouds"""
    
    print("\n" + "="*60)
    print("ONNX Semantic Segmentation Inference Test")
    print("="*60 + "\n")
    
    # Initialize
    inferencer = SemanticSegmentationInferencer(model_path, num_points=4096)
    
    # Generate synthetic test clouds
    print(f"Generating {num_test_samples} test point clouds...\n")
    for test_id in range(num_test_samples):
        # Random point cloud: x, y, z, intensity
        points = np.random.randn(4096, 4).astype(np.float32)
        points[:, :3] = np.random.uniform(-5, 5, (4096, 3))  # x, y, z
        points[:, 3] = np.random.uniform(20, 60, 4096)  # intensity
        
        # Run inference
        class_ids, confidences = inferencer.infer(points)
        
        # Statistics
        unique_classes = np.unique(class_ids)
        class_counts = {
            cid: (class_ids == cid).sum()
            for cid in unique_classes
        }
        
        avg_conf = confidences.mean()
        
        print(f"Test {test_id+1}:")
        print(f"  Points: {len(points)}")
        print(f"  Classes predicted: {unique_classes}")
        for cid, count in sorted(class_counts.items()):
            pct = 100 * count / len(points)
            class_name = SemanticSegmentationInferencer.CLASS_NAMES.get(cid, "UNKNOWN")
            print(f"    {class_name:20s} ({cid}): {count:5d} ({pct:5.1f}%)")
        print(f"  Avg confidence: {avg_conf:.4f}")
        print()
    
    print("="*60)
    print("✓ Inference test passed successfully!")
    print("="*60)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test ONNX semantic segmentation model')
    parser.add_argument('--model', type=str, default='./semantic_model.onnx',
                       help='Path to ONNX model')
    parser.add_argument('--num-tests', type=int, default=5,
                       help='Number of test point clouds')
    
    args = parser.parse_args()
    
    try:
        test_inference(args.model, args.num_tests)
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
