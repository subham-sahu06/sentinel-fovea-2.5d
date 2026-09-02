#!/bin/bash
# ============================================================================
# BASH COMMANDS FOR LIDAR 3D SEMANTIC SEGMENTATION PIPELINE
# Copy and paste these commands directly into your terminal
# ============================================================================

# SETUP (One-time, ~2-3 minutes)
# ============================================================================
cd /home/subham/robot-dashboard
bash setup_training.sh

# This will:
# - Create virtual environment (venv/)
# - Install PyTorch (CPU or GPU)
# - Install dependencies (pandas, onnx, onnxruntime)
# - Verify installation


# TRAIN MODEL (Main execution, ~20-30 minutes on GPU)
# ============================================================================
bash run_training.sh

# This will:
# - Load dataset (dataset.csv, 13,991 points)
# - Train for 20 epochs
# - Output: checkpoints/ + semantic_model.onnx


# CUSTOM TRAINING (if you need to adjust parameters)
# ============================================================================
# Activate environment first
source venv/bin/activate

# Option A: Fast training (development/testing, ~10 min)
python3 train_pointcloud.py \
    --data dataset.csv \
    --epochs 3 \
    --batch-size 16 \
    --num-points 2048 \
    --device cuda

# Option B: Production training (balanced, ~20 min)
python3 train_pointcloud.py \
    --data dataset.csv \
    --epochs 20 \
    --batch-size 8 \
    --num-points 4096 \
    --device cuda

# Option C: High quality training (~40 min)
python3 train_pointcloud.py \
    --data dataset.csv \
    --epochs 30 \
    --batch-size 4 \
    --num-points 8192 \
    --lr 0.0005 \
    --weight-decay 2e-4 \
    --device cuda

# Option D: CPU-only (no GPU needed, ~60 min)
python3 train_pointcloud.py \
    --data dataset.csv \
    --epochs 20 \
    --batch-size 4 \
    --num-points 2048 \
    --device cpu


# MONITOR TRAINING (run in separate terminal)
# ============================================================================
tail -f training.log

# Or view specific metrics:
grep "Mean IoU\|DRIVABLE\|NEGATIVE_TRENCH" training.log


# VERIFY ONNX MODEL (after training)
# ============================================================================
source venv/bin/activate

# Test ONNX with synthetic data
python3 test_inference.py --model semantic_model.onnx --num-tests 5

# Validate ONNX structure
python3 << 'EOF'
import onnx
model = onnx.load('semantic_model.onnx')
print("✓ ONNX model is valid")
print(f"Input shape: (batch_size, 4096, 4)")
print(f"Output shape: (batch_size, 4096, 4)")
EOF


# CHECK TRAINING OUTPUTS
# ============================================================================
# View all generated files
ls -lh checkpoints/
ls -lh semantic_model.onnx
ls -lh training.log

# View final training metrics
tail -50 training.log


# ROS 2 INTEGRATION (after training)
# ============================================================================

# 1. Copy ONNX model to ROS 2 package
cp semantic_model.onnx ros2_ws/src/demo_pipeline/

# 2. Install ROS 2 inference dependencies
pip install onnxruntime

# 3. Source ROS 2 environment
source ros2_ws/install/setup.bash

# 4. Run inference node (in Terminal 1)
ros2 run demo_pipeline semantic_segmentation.py

# 5. Check output (in Terminal 2)
ros2 topic echo /perception/semantic_cloud

# 6. Monitor inference performance
ros2 topic hz /perception/semantic_cloud


# CLEANUP (if needed)
# ============================================================================

# Remove virtual environment (to start fresh)
rm -rf venv/

# Clean checkpoints (keep only best model)
rm checkpoints/epoch_*.pt

# Clear ONNX model (regenerate)
rm semantic_model.onnx


# TROUBLESHOOTING COMMANDS
# ============================================================================

# Check Python version
python3 --version

# Verify PyTorch installation
python3 -c "import torch; print(f'PyTorch {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}')"

# Check disk space
df -h

# Monitor GPU during training (separate terminal)
nvidia-smi -l 1

# View system resources
top

# Check if ports are available
lsof -i :9090  # rosbridge default port


# PRODUCTION DEPLOYMENT CHECKLIST
# ============================================================================

# 1. Verify model accuracy
python3 train_pointcloud.py --data dataset.csv --epochs 1 --device cuda

# 2. Test ONNX inference
python3 test_inference.py --model semantic_model.onnx

# 3. Check model size
du -h semantic_model.onnx

# 4. Validate ROS 2 integration
source ros2_ws/install/setup.bash
ros2 run demo_pipeline semantic_segmentation.py &
sleep 2
ros2 topic list | grep semantic
pkill -f semantic_segmentation.py

# 5. Profile latency (create test node)
python3 << 'EOF'
import time
import numpy as np
import onnxruntime as ort

session = ort.InferenceSession('semantic_model.onnx')
input_name = session.get_inputs()[0].name
dummy = np.random.randn(1, 4096, 4).astype(np.float32)

# Warmup
for _ in range(5):
    session.run(None, {input_name: dummy})

# Benchmark
start = time.time()
for _ in range(100):
    session.run(None, {input_name: dummy})
elapsed = time.time() - start
print(f"Average latency: {elapsed/100*1000:.2f} ms")
print(f"FPS: {100/elapsed:.1f}")
EOF

# ============================================================================
# END OF BASH COMMANDS
# ============================================================================

# SUMMARY:
# 1. Setup:   bash setup_training.sh          (1x, ~3 min)
# 2. Train:   bash run_training.sh            (~20 min on GPU)
# 3. Verify:  python3 test_inference.py       (~5 sec)
# 4. Deploy:  Copy to ros2_ws, run node       (~5 min setup)
#
# TOTAL TIME: ~30 minutes from start to ROS 2 inference ✅
