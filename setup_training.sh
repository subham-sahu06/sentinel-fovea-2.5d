#!/bin/bash
# Setup and training orchestration script
# Installs dependencies and runs training pipeline

set -e  # Exit on error

echo "================================"
echo "LiDAR Semantic Segmentation Setup"
echo "================================"

# Detect Python
PYTHON=$(command -v python3 || command -v python)
echo "Using Python: $PYTHON"
$PYTHON --version

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo ""
    echo "[1/4] Creating Python virtual environment..."
    $PYTHON -m venv venv
    source venv/bin/activate
else
    echo ""
    echo "[1/4] Using existing virtual environment..."
    source venv/bin/activate
fi

# Upgrade core packaging tools
echo ""
echo "[2/4] Upgrading pip and packaging tools..."
pip install --upgrade pip setuptools wheel

# Install dependencies from requirements
echo ""
echo "[3/4] Installing PyTorch and ML dependencies..."
pip install -r requirements-training.txt

# Verify installation
echo ""
echo "[4/4] Verifying installation..."
python -c "import torch; print(f'PyTorch {torch.__version__} (CUDA available: {torch.cuda.is_available()})')"
python -c "import onnx; print(f'ONNX {onnx.__version__}')"
python -c "import onnxruntime; print(f'ONNX Runtime {onnxruntime.__version__}')"
python -c "import pandas; print(f'Pandas {pandas.__version__}')"

echo ""
echo "================================"
echo "✓ Setup complete!"
echo "================================"
echo ""
echo "To start training, run:"
echo "  bash run_training.sh"
echo ""
