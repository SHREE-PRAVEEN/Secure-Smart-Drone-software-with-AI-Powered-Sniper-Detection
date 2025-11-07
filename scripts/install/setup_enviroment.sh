#!/usr/bin/env bash
# ============================================================
# MAYURI - Environment Setup Script
# Author: MAYURI Robotics Team
# Date: 2025-10-22
#
# Description:
#   Unified one-click setup script for the MAYURI robotics system.
#   It installs all dependencies, ROS2, AI frameworks, and sets
#   up the environment variables for development or deployment.
#
# Usage:
#   chmod +x setup_environment.sh
#   ./setup_environment.sh
#
# ============================================================

set -e  # Exit immediately if any command fails

echo "============================================"
echo "     🚀 MAYURI Environment Setup"
echo "============================================"

# ------------------------------------------------------------
# 🧩 Detect OS & Platform
# ------------------------------------------------------------
ARCH=$(uname -m)
UBUNTU_VERSION=$(lsb_release -rs)
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
echo "🧠 Ubuntu Version: $UBUNTU_VERSION"
sleep 1

# ------------------------------------------------------------
# ⚙️ Create Virtual Environment
# ------------------------------------------------------------
ENV_PATH=~/mayuri_env
if [ ! -d "$ENV_PATH" ]; then
    echo "🐍 Creating Python virtual environment at: $ENV_PATH"
    python3 -m venv $ENV_PATH
else
    echo "✅ Virtual environment already exists."
fi

# Activate environment
source $ENV_PATH/bin/activate

# ------------------------------------------------------------
# 📦 Step 1: Install System Dependencies
# ------------------------------------------------------------
echo "⚙️ Step 1/4: Installing System Dependencies..."
bash scripts/install/install_dependencies.sh

# ------------------------------------------------------------
# 🤖 Step 2: Install AI Frameworks
# ------------------------------------------------------------
echo "🧠 Step 2/4: Installing AI Frameworks..."
bash scripts/install/install_ai_frameworks.sh

# ------------------------------------------------------------
# 🦾 Step 3: Install ROS2
# ------------------------------------------------------------
echo "🛰️ Step 3/4: Installing ROS2..."
bash scripts/install/install_ros2.sh

# ------------------------------------------------------------
# 🧠 Step 4: Generate Hardware Configuration
# ------------------------------------------------------------
echo "🔧 Step 4/4: Running Hardware Detection..."
python3 tools/hardware_setup.py --profile auto_detected

# ------------------------------------------------------------
# 🌍 Setup Environment Variables
# ------------------------------------------------------------
echo "🌍 Setting up environment variables..."

ROS_DISTRO="humble"
if [[ "$UBUNTU_VERSION" == "24.04" ]]; then
    ROS_DISTRO="iron"
fi

{
    echo ""
    echo "# ==========================================="
    echo "# MAYURI Environment Variables"
    echo "# ==========================================="
    echo "export MAYURI_HOME=$(pwd)"
    echo "export MAYURI_ENV=$ENV_PATH"
    echo "export ROS_DISTRO=$ROS_DISTRO"
    echo "export ROS_VERSION=2"
    echo "export ROS_DOMAIN_ID=7"
    echo "export ROS_NAMESPACE=mayuri"
    echo "export PYTHONPATH=\$MAYURI_HOME:\$PYTHONPATH"
    echo "source /opt/ros/\$ROS_DISTRO/setup.bash"
    echo "source ~/mayuri_ws/install/setup.bash"
    echo "export PATH=\$MAYURI_ENV/bin:\$PATH"
} >> ~/.bashrc

# ------------------------------------------------------------
# 🧪 Verification Step
# ------------------------------------------------------------
echo "🧪 Verifying installation..."
source ~/.bashrc
python3 --version
ros2 --version || echo "⚠️ ROS2 verification pending (open a new terminal to apply environment variables)."

# ------------------------------------------------------------
# ✅ Final Summary
# ------------------------------------------------------------
echo ""
echo "============================================"
echo "✅ MAYURI Environment Setup Completed!"
echo "============================================"
echo "📁 Virtual Environment: $ENV_PATH"
echo "🧠 ROS2 Workspace: ~/mayuri_ws"
echo "🔐 Config Directory: mayuri/config/"
echo "🎯 Next Steps:"
echo "   1️⃣ Activate your environment: source ~/.bashrc"
echo "   2️⃣ Test hardware: ros2 launch mayuri test_hardware.launch.py"
echo "   3️⃣ Start full system: ros2 launch mayuri full_system.launch.py"
echo "============================================"
