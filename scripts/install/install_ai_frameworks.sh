#!/usr/bin/env bash
# ============================================================
# MAYURI - AI Frameworks Installation Script
# Author: MAYURI Robotics Team
# Date: 2025-10-22
# 
# Description:
#   Installs deep learning frameworks (PyTorch, TensorFlow Lite,
#   ONNX Runtime, OpenCV, and supporting dependencies).
#
# Supported Platforms:
#   - NVIDIA Jetson (Nano, Xavier, Orin)
#   - Raspberry Pi (4/5)
#   - Intel NUC
#   - x86_64 Desktops/Laptops
# ============================================================

set -e  # Exit immediately if a command fails

echo "============================================"
echo "   MAYURI AI Frameworks Installer"
echo "============================================"

# ------------------------------------------------------------
# 🧩 Detect Platform
# ------------------------------------------------------------
ARCH=$(uname -m)
PLATFORM="unknown"

if [[ "$ARCH" == "aarch64" ]]; then
    if [[ -f "/etc/nv_tegra_release" ]]; then
        PLATFORM="jetson"
    elif grep -qi "raspberry" /proc/device-tree/model 2>/dev/null; then
        PLATFORM="raspberry_pi"
    else
        PLATFORM="arm_edge"
    fi
else
    PLATFORM="x86_64"
fi

echo "🔍 Detected Platform: $PLATFORM"
sleep 1

# ------------------------------------------------------------
# 🧠 Update and Install Base Packages
# ------------------------------------------------------------
echo "📦 Updating system and installing base tools..."
sudo apt update && sudo apt install -y \
    python3 python3-pip python3-dev python3-venv \
    libopenblas-dev liblapack-dev libjpeg-dev zlib1g-dev \
    libgtk-3-dev libboost-all-dev cmake git wget unzip curl

python3 -m pip install --upgrade pip setuptools wheel

# ------------------------------------------------------------
# ⚙️ Install Frameworks
# ------------------------------------------------------------

install_pytorch() {
    echo "🔥 Installing PyTorch..."
    if [[ "$PLATFORM" == "jetson" ]]; then
        echo "Installing PyTorch optimized for Jetson..."
        wget https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.1.0+nv24.06-cp38-cp38-linux_aarch64.whl -O torch.whl
        python3 -m pip install torch.whl torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu118
    else
        python3 -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
    fi
}

install_tensorflow_lite() {
    echo "🤖 Installing TensorFlow Lite runtime..."
    if [[ "$PLATFORM" == "raspberry_pi" || "$PLATFORM" == "arm_edge" ]]; then
        python3 -m pip install tflite-runtime
    else
        python3 -m pip install tensorflow==2.15.0
    fi
}

install_onnx_runtime() {
    echo "🧩 Installing ONNX Runtime..."
    if [[ "$PLATFORM" == "jetson" ]]; then
        python3 -m pip install onnxruntime-gpu
    else
        python3 -m pip install onnxruntime
    fi
}

install_opencv() {
    echo "🎥 Installing OpenCV..."
    python3 -m pip install opencv-python==4.10.0.84 opencv-contrib-python==4.10.0.84
}

install_extras() {
    echo "⚡ Installing extra utilities..."
    python3 -m pip install numpy scipy pandas matplotlib pillow tqdm pyyaml GPUtil psutil
}

# ------------------------------------------------------------
# 🚀 Run Installations
# ------------------------------------------------------------
install_pytorch
install_tensorflow_lite
install_onnx_runtime
install_opencv
install_extras

# ------------------------------------------------------------
# ✅ Verification
# ------------------------------------------------------------
echo "🧪 Verifying installations..."
python3 - <<'EOF'
import torch, onnxruntime, cv2, numpy, platform
print("PyTorch:", torch.__version__, "| CUDA:", torch.cuda.is_available())
print("ONNX Runtime:", onnxruntime.__version__)
print("OpenCV:", cv2.__version__)
print("Platform:", platform.machine())
EOF

# ------------------------------------------------------------
# 📂 Finalization
# ------------------------------------------------------------
echo "✅ All AI frameworks installed successfully!"
echo "🧠 You can now run: python3 tools/hardware_setup.py"
echo "============================================"
