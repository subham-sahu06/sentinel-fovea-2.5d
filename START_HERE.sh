#!/bin/bash
# ============================================================================
# START HERE - RUN THIS COMMAND NOW
# ============================================================================

# Copy and paste this exact command into your terminal:

cd /home/subham/robot-dashboard && bash setup_training.sh && bash run_training.sh

# ============================================================================
# This will:
#   1. ✅ Create Python virtual environment
#   2. ✅ Install PyTorch + dependencies
#   3. ✅ Train for 20 epochs
#   4. ✅ Export semantic_model.onnx for ROS 2
#
# Expected runtime: 20-30 minutes on GPU (60 min on CPU)
# Expected output: 
#   - checkpoints/best_model.pt
#   - semantic_model.onnx (ready for ROS 2)
#   - training.log (complete metrics)
# ============================================================================

# Monitor training in real-time (optional, in separate terminal):
tail -f /home/subham/robot-dashboard/training.log

# After training completes, verify ONNX model:
python3 /home/subham/robot-dashboard/test_inference.py --model /home/subham/robot-dashboard/semantic_model.onnx

# Deploy to ROS 2 (optional):
cp /home/subham/robot-dashboard/semantic_model.onnx ~/robot-dashboard/ros2_ws/src/demo_pipeline/
source ~/robot-dashboard/ros2_ws/install/setup.bash
ros2 run demo_pipeline semantic_segmentation.py

# ============================================================================
# For more details:
#   - QUICK_START_TRAINING.md
#   - BASH_COMMANDS.sh
#   - TRAINING_GUIDE.md
# ============================================================================
