#!/usr/bin/env bash
# ============================================================
# MAYURI - System Dependencies Installation Script
# Author: MAYURI Robotics Team
# Date: 2025-10-22
#
# Description:
#   Installs all core system dependencies for the MAYURI
#   surveillance and defense robotics platform.
#
# Includes:
#   - ROS2 Base
#   - Docker & Compose
#   - ZeroMQ, OpenSSL, and libzmq
#   - FFmpeg, GStreamer
#   - Python build tools and utilities
# ============================================================

set -e  # Stop on any error

echo "============================================"
echo "   MAYURI System Dependencies Installer"
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
# ⚙️ Update System and Core Utilities
# ------------------------------------------------------------
echo "📦 Updating system packages..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    build-essential cmake git curl wget unzip zip pkg-config \
    python3 python3-pip python3-venv python3-dev python3-yaml \
    net-tools htop screen nano vim lsb-release gnupg2 ca-certificates

echo "✅ Base packages installed."

# ------------------------------------------------------------
# 🧠 Install ROS2 (Humble for Ubuntu 22.04)
# ------------------------------------------------------------
if ! command -v ros2 &> /dev/null; then
    echo "🚀 Installing ROS2 Humble Desktop..."
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
    sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
    sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
else
    echo "✅ ROS2 already installed."
fi

# ------------------------------------------------------------
# 🐳 Install Docker & Docker Compose
# ------------------------------------------------------------
if ! command -v docker &> /dev/null; then
    echo "🐳 Installing Docker..."
    sudo apt install -y apt-transport-https ca-certificates curl gnupg-agent software-properties-common
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
    sudo add-apt-repository \
        "deb [arch=$(dpkg --print-architecture)] https://download.docker.com/linux/ubuntu \
        $(lsb_release -cs) stable"
    sudo apt update && sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
    sudo usermod -aG docker $USER
else
    echo "✅ Docker already installed."
fi

if ! command -v docker-compose &> /dev/null; then
    echo "📦 Installing Docker Compose..."
    sudo apt install -y docker-compose
else
    echo "✅ Docker Compose already installed."
fi

# ------------------------------------------------------------
# 🔒 Install Networking, Security & Communication Libraries
# ------------------------------------------------------------
echo "🔐 Installing OpenSSL, ZeroMQ, and related libraries..."
sudo apt install -y \
    libssl-dev libzmq3-dev libffi-dev libprotobuf-dev protobuf-compiler \
    libsodium-dev uuid-dev

# ------------------------------------------------------------
# 🎥 Install Multimedia & Streaming Libraries
# ------------------------------------------------------------
echo "🎥 Installing FFmpeg and GStreamer..."
sudo apt install -y ffmpeg gstreamer1.0-tools gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav \
    libavcodec-dev libavformat-dev libswscale-dev

# ------------------------------------------------------------
# 🧰 Python and Dev Tools
# ------------------------------------------------------------
echo "🐍 Installing Python build tools..."
python3 -m pip install --upgrade pip setuptools wheel
python3 -m pip install numpy pyzmq cryptography requests psutil tqdm rich

# ------------------------------------------------------------
# 🧩 Jetson / Raspberry Pi Specific Optimizations
# ------------------------------------------------------------
if [[ "$PLATFORM" == "jetson" ]]; then
    echo "🧠 Applying Jetson optimizations..."
    sudo apt install -y nvidia-jetpack
    python3 -m pip install jetson-stats
elif [[ "$PLATFORM" == "raspberry_pi" ]]; then
    echo "🍓 Applying Raspberry Pi optimizations..."
    sudo apt install -y python3-rpi.gpio python3-picamera2
fi

# ------------------------------------------------------------
# 🧪 Verification
# ------------------------------------------------------------
echo "🔎 Verifying key installations..."
for cmd in python3 pip ros2 docker ffmpeg openssl; do
    if command -v $cmd &> /dev/null; then
        echo "✅ $cmd installed."
    else
        echo "❌ $cmd not found!"
    fi
done

# ------------------------------------------------------------
# 📂 Finalization
# ------------------------------------------------------------
echo "============================================"
echo "✅ All system dependencies installed successfully!"
echo "🧠 You can now run:"
echo "   ./scripts/install/install_ai_frameworks.sh"
echo "============================================"
