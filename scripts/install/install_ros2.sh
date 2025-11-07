#!/usr/bin/env bash
# ============================================================
# MAYURI - ROS2 Installation Script
# Author: MAYURI Robotics Team
# Date: 2025-10-22
#
# Description:
#   Installs ROS2 (Humble for Ubuntu 22.04 or Iron for Ubuntu 24.04)
#   along with essential development tools and environment setup
#   for the MAYURI robotics framework.
#
# Supported Platforms:
#   - Ubuntu 22.04 (ROS2 Humble)
#   - Ubuntu 24.04 (ROS2 Iron)
#   - NVIDIA Jetson, Raspberry Pi, Intel NUC, Desktop
# ============================================================

set -e  # Exit on any error

echo "============================================"
echo "   MAYURI ROS2 Installer"
echo "============================================"

# ------------------------------------------------------------
# 🧩 Detect Ubuntu Version
# ------------------------------------------------------------
UBUNTU_VERSION=$(lsb_release -rs)
UBUNTU_CODENAME=$(lsb_release -sc)

if [[ "$UBUNTU_VERSION" == "22.04" ]]; then
    ROS_DISTRO="humble"
elif [[ "$UBUNTU_VERSION" == "24.04" ]]; then
    ROS_DISTRO="iron"
else
    echo "⚠️ Unsupported Ubuntu version: $UBUNTU_VERSION"
    echo "Supported versions: 22.04 (Humble), 24.04 (Iron)"
    exit 1
fi

echo "🔍 Detected Ubuntu $UBUNTU_VERSION ($UBUNTU_CODENAME)"
echo "🚀 Installing ROS2 $ROS_DISTRO ..."
sleep 1

# ------------------------------------------------------------
# 🧠 Add ROS2 Repository & Keys
# ------------------------------------------------------------
sudo apt update && sudo apt install -y curl gnupg2 lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -

sudo sh -c "echo 'deb http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main' > /etc/apt/sources.list.d/ros2-latest.list"

sudo apt update

# ------------------------------------------------------------
# ⚙️ Install ROS2 Base or Desktop
# ------------------------------------------------------------
echo "📦 Installing ROS2 packages..."
sudo apt install -y ros-$ROS_DISTRO-desktop \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool build-essential

# ------------------------------------------------------------
# 🧰 Initialize rosdep
# ------------------------------------------------------------
echo "🔧 Initializing rosdep..."
sudo rosdep init || true
rosdep update

# ------------------------------------------------------------
# 🧩 Setup Environment
# ------------------------------------------------------------
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
WORKSPACE=~/mayuri_ws

echo "🧠 Setting up ROS2 workspace at: $WORKSPACE"
mkdir -p $WORKSPACE/src
cd $WORKSPACE

# Add environment sourcing to .bashrc
if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source ~/mayuri_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/mayuri_ws/install/setup.bash" >> ~/.bashrc
fi

source $ROS_SETUP

# ------------------------------------------------------------
# 🧪 Test Installation
# ------------------------------------------------------------
echo "🧪 Verifying ROS2 installation..."
if command -v ros2 &> /dev/null; then
    echo "✅ ROS2 command found!"
    ros2 --version
else
    echo "❌ ROS2 not found in PATH!"
    exit 1
fi

# ------------------------------------------------------------
# 📂 Build Empty Workspace
# ------------------------------------------------------------
echo "⚙️ Building initial ROS2 workspace..."
colcon build --symlink-install || echo "⚠️ Build skipped (empty src folder)."

# ------------------------------------------------------------
# 🎯 Summary
# ------------------------------------------------------------
echo "============================================"
echo "✅ ROS2 $ROS_DISTRO installation completed!"
echo "📁 Workspace: $WORKSPACE"
echo "🧠 Environment setup in: ~/.bashrc"
echo "🔹 To activate now, run: source ~/.bashrc"
echo "============================================"
