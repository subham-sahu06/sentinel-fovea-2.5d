#!/bin/bash
# Training execution script
# Runs the complete semantic segmentation training pipeline

set -e  # Exit on error

# Activate virtual environment if it exists
if [ -f "venv/bin/activate" ]; then
    echo "Activating virtual environment..."
    source venv/bin/activate
fi

# Detect Device (CUDA GPU if available, else CPU)
if python3 -c "import torch; exit(0 if torch.cuda.is_available() else 1)" 2>/dev/null; then
    DEVICE="cuda"
else
    DEVICE="cpu"
fi

# Configuration
DATA_PATH="dataset.csv"
EPOCHS=15
BATCH_SIZE=8
NUM_POINTS=2048
LEARNING_RATE=0.001
CHECKPOINT_DIR="./checkpoints"
ONNX_OUTPUT="semantic_model.onnx"

echo "================================"
echo "Training Semantic Segmentation"
echo "================================"
echo "Dataset: $DATA_PATH"
echo "Epochs: $EPOCHS"
echo "Batch Size: $BATCH_SIZE"
echo "Points per Cloud: $NUM_POINTS"
echo "Learning Rate: $LEARNING_RATE"
echo "Device: $DEVICE"
echo "Checkpoints: $CHECKPOINT_DIR"
echo "ONNX Output: $ONNX_OUTPUT"
echo "================================"
echo ""

# Create checkpoint directory
mkdir -p "$CHECKPOINT_DIR"

# Run training
echo "Starting training pipeline..."
python3 train_pointcloud.py \
    --data "$DATA_PATH" \
    --epochs "$EPOCHS" \
    --batch-size "$BATCH_SIZE" \
    --num-points "$NUM_POINTS" \
    --lr "$LEARNING_RATE" \
    --device "$DEVICE" \
    --checkpoint-dir "$CHECKPOINT_DIR" \
    --export-onnx "$ONNX_OUTPUT" \
    --seed 42

echo ""
echo "================================"
echo "✓ Training complete!"
echo "================================"
echo ""
echo "Outputs:"
echo "  - Checkpoints: $CHECKPOINT_DIR/"
echo "  - ONNX Model: $ONNX_OUTPUT"
echo "  - Log File: training.log"
echo ""
echo "Next steps:"
echo "  1. Model ready for ROS 2 inference at $ONNX_OUTPUT"
echo "  2. Run inference with: ros2 run demo_pipeline semantic_segmentation"
echo ""
