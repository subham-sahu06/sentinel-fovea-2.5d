# Quick Start: Training LiDAR Semantic Segmentation

## One-Command Training (Recommended)

```bash
cd /home/subham/robot-dashboard && bash setup_training.sh && bash run_training.sh
```

This single command:
1. ✅ Creates Python virtual environment
2. ✅ Installs PyTorch and dependencies
3. ✅ Runs 20-epoch training on your dataset
4. ✅ Generates `semantic_model.onnx` for ROS 2

---

## Step-by-Step Commands

### Step 1: Setup Environment (one-time only)
```bash
cd /home/subham/robot-dashboard
bash setup_training.sh
```

Output:
```
================================
LiDAR Semantic Segmentation Setup
================================
Using Python: /usr/bin/python3
Python 3.10.12

[1/4] Creating Python virtual environment...
[2/4] Installing PyTorch...
[3/4] Installing dependencies...
[4/4] Verifying installation...
PyTorch 2.1.2
ONNX 1.15.0
Pandas 2.1.3

================================
✓ Setup complete!
================================

To start training, run:
  bash run_training.sh
```

### Step 2: Run Training
```bash
bash run_training.sh
```

Training console output (excerpt):
```
================================
Training Semantic Segmentation
================================
Dataset: dataset.csv
Epochs: 20
Batch Size: 8
Points per Cloud: 4096
Learning Rate: 0.001
Device: cuda
Checkpoint Dir: ./checkpoints
ONNX Output: semantic_model.onnx
================================

Loaded 13991 points from dataset.csv
Class distribution: {'DRIVABLE': 7500, 'NEGATIVE_TRENCH': 1200, ...}
Total frames: 1
Model parameters: 2,456,324

--- Epoch [1/20] ---
  Batch [10/11] Loss: 1.2345
Train Loss: 1.1234
Val Loss: 1.0876
Mean IoU: 0.4523
  DRIVABLE (ID 0): 0.7821
  NEGATIVE_TRENCH (ID 1): 0.3456
  STATIC_OBSTACLE (ID 2): 0.4234
  DYNAMIC_TARGET (ID 3): 0.1234
✓ Saved best model with mIoU 0.4523

--- Epoch [2/20] ---
...
```

---

## Full Training Command (Manual)

If you prefer to avoid the shell scripts:

```bash
cd /home/subham/robot-dashboard

# Activate environment
source venv/bin/activate

# Run training with all options
python3 train_pointcloud.py \
    --data dataset.csv \
    --epochs 20 \
    --batch-size 8 \
    --num-points 4096 \
    --lr 0.001 \
    --weight-decay 1e-4 \
    --device cuda \
    --checkpoint-dir ./checkpoints \
    --export-onnx ./semantic_model.onnx \
    --seed 42
```

---

## After Training: Output Files

```bash
# Check generated files
ls -lh checkpoints/
ls -lh semantic_model.onnx
cat training.log | tail -20  # View final training metrics
```

Expected files:
- `checkpoints/best_model.pt` - Best trained model (highest mIoU)
- `checkpoints/epoch_1.pt` through `epoch_20.pt` - Per-epoch checkpoints
- `semantic_model.onnx` - ONNX model for ROS 2 inference
- `training.log` - Complete training log

---

## Verify ONNX Model

```bash
python3 << 'EOF'
import onnx
model = onnx.load('semantic_model.onnx')
print("✓ ONNX model is valid")
print(f"Inputs: {[inp.name for inp in model.graph.input]}")
print(f"Outputs: {[out.name for out in model.graph.output]}")
print(f"Input shape: (batch_size, 4096, 4)")
print(f"Output shape: (batch_size, 4096, 4)")
EOF
```

---

## Integrate with ROS 2

Once training is complete:

```bash
# Copy ONNX model to ROS 2 package
cp semantic_model.onnx ~/robot-dashboard/ros2_ws/src/demo_pipeline/

# Source ROS 2 environment
source ~/robot-dashboard/ros2_ws/install/setup.bash

# Run inference node (in separate terminal)
ros2 run demo_pipeline semantic_segmentation.py

# In another terminal, verify output:
ros2 topic echo /perception/semantic_cloud
```

---

## Configuration Presets

### Fast Training (Development)
```bash
python3 train_pointcloud.py \
    --data dataset.csv \
    --epochs 3 \
    --batch-size 16 \
    --num-points 2048 \
    --device cuda
```

### Balanced (Production)
```bash
python3 train_pointcloud.py \
    --data dataset.csv \
    --epochs 20 \
    --batch-size 8 \
    --num-points 4096 \
    --device cuda
```

### High Quality (Longer Training)
```bash
python3 train_pointcloud.py \
    --data dataset.csv \
    --epochs 30 \
    --batch-size 4 \
    --num-points 8192 \
    --lr 0.0005 \
    --weight-decay 2e-4 \
    --device cuda
```

### CPU-Only (No GPU)
```bash
python3 train_pointcloud.py \
    --data dataset.csv \
    --epochs 20 \
    --batch-size 4 \
    --num-points 2048 \
    --device cpu
```

---

## Monitoring Training

In a separate terminal, monitor the log file in real-time:

```bash
tail -f training.log
```

Or view final metrics:

```bash
grep "Mean IoU\|DRIVABLE\|NEGATIVE_TRENCH\|STATIC_OBSTACLE\|DYNAMIC_TARGET" training.log
```

---

## Troubleshooting

### Out of Memory (GPU)
```bash
# Reduce batch size
python3 train_pointcloud.py --batch-size 2 --num-points 2048
```

### Module not found errors
```bash
# Reinstall dependencies
source venv/bin/activate
pip install -r requirements-training.txt --force-reinstall
```

### ONNX export fails
```bash
# Update ONNX tools
pip install --upgrade onnx onnxruntime
```

---

## Complete Pipeline Summary

```
1. Setup:           bash setup_training.sh         (1x, ~2 min)
2. Train:           bash run_training.sh           (~20 min on GPU)
3. Outputs:         checkpoints/ + semantic_model.onnx
4. Verify:          python3 -c "import onnx; onnx.load('semantic_model.onnx')"
5. ROS 2 Deploy:    Copy .onnx to ros2_ws, run semantic_segmentation.py node
```

**Total time from start to ROS 2 inference: ~25-30 minutes on GPU**

---

## Dataset Classes Reference

| ID | Name | Example |
|-----|------|---------|
| 0 | DRIVABLE | Road surface, safe to drive |
| 1 | NEGATIVE_TRENCH | Holes, dropoffs, cliffs |
| 2 | STATIC_OBSTACLE | Parked cars, poles, walls |
| 3 | DYNAMIC_TARGET | Moving pedestrians, vehicles |

---

**For detailed docs, see: [TRAINING_GUIDE.md](TRAINING_GUIDE.md)**
