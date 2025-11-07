"""
system_installer.py
------------------------------------------------------------
MAYURI - Automated System Installer
Author: MAYURI Robotics Team
Date: 2025-10-22

Description:
    Auto-installs core dependencies and environment setup
    for MAYURI based on hardware profile (from hardware_setup.py).

    Tasks:
        - Detect platform (Jetson, Pi, NUC, Desktop)
        - Install ROS2 & Python dependencies
        - Install CUDA/TensorRT (if supported)
        - Create environment directories
        - Verify successful setup
------------------------------------------------------------
"""

import os
import sys
import yaml
import subprocess
from pathlib import Path
import platform
from datetime import datetime


# ------------------------------------------------------------
# 🧩 Helper Utilities
# ------------------------------------------------------------

def run_command(cmd, sudo=False):
    """Run a shell command with optional sudo."""
    prefix = "sudo " if sudo and os.geteuid() != 0 else ""
    full_cmd = prefix + cmd
    print(f"🔹 Running: {full_cmd}")
    result = subprocess.run(full_cmd, shell=True)
    if result.returncode != 0:
        print(f"❌ Command failed: {cmd}")
        sys.exit(1)


def load_hardware_config():
    """Find and load latest hardware configuration YAML."""
    config_dir = Path("config/hardware")
    if not config_dir.exists():
        print("❌ No hardware config found. Run hardware_setup.py first.")
        sys.exit(1)

    configs = sorted(config_dir.glob("*.yaml"), key=os.path.getmtime, reverse=True)
    if not configs:
        print("❌ No YAML hardware profile found.")
        sys.exit(1)

    latest = configs[0]
    print(f"📁 Using hardware profile: {latest.name}")
    with open(latest, "r") as f:
        return yaml.safe_load(f)


def is_command_available(command):
    """Check if a shell command is available."""
    return subprocess.call(f"type {command}", shell=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0


# ------------------------------------------------------------
# ⚙️ Installation Steps
# ------------------------------------------------------------

def install_python_dependencies():
    """Install Python libraries from requirements.txt."""
    if not Path("requirements.txt").exists():
        print("❌ Missing requirements.txt file.")
        sys.exit(1)

    print("\n📦 Installing Python dependencies...")
    run_command("python3 -m pip install --upgrade pip")
    run_command("pip install -r requirements.txt")


def install_ros2():
    """Install ROS2 (Humble or Iron) depending on Ubuntu version."""
    ubuntu_version = subprocess.getoutput("lsb_release -sc").strip()
    print(f"\n🧠 Detected Ubuntu version: {ubuntu_version}")

    if not is_command_available("ros2"):
        print("🚀 Installing ROS2 Humble (if not installed)...")
        run_command(
            "sudo apt update && "
            "sudo apt install -y curl gnupg2 lsb-release software-properties-common"
        )
        run_command(
            "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -"
        )
        run_command(
            "sudo sh -c 'echo \"deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros2-latest.list'"
        )
        run_command("sudo apt update && sudo apt install -y ros-humble-desktop")
        print("✓ ROS2 installed successfully.")
    else:
        print("✓ ROS2 already installed.")


def install_cuda_toolkit():
    """Install CUDA toolkit if supported."""
    if "NVIDIA" not in platform.uname().version and not Path("/usr/local/cuda").exists():
        print("⚠️ CUDA not supported or NVIDIA GPU not detected.")
        return

    print("🚀 Installing NVIDIA CUDA Toolkit (if not installed)...")
    if not Path("/usr/local/cuda").exists():
        run_command("sudo apt install -y nvidia-cuda-toolkit")
    else:
        print("✓ CUDA Toolkit already installed.")


def install_tensor_rt():
    """Install TensorRT on Jetson or NVIDIA systems."""
    if "jetson" not in platform.node().lower() and "nvidia" not in platform.uname().version.lower():
        print("⚠️ TensorRT not supported on this platform.")
        return

    print("🚀 Installing NVIDIA TensorRT Runtime...")
    run_command("sudo apt install -y tensorrt libnvinfer-bin libnvinfer-dev")


def create_directory_structure():
    """Create essential folders (logs, models, configs)."""
    print("\n📁 Creating project directories...")
    dirs = [
        "logs/ai",
        "logs/missions",
        "data/models",
        "data/telemetry",
        "backups/configs",
    ]
    for d in dirs:
        Path(d).mkdir(parents=True, exist_ok=True)
    print("✓ Directory structure ready.")


def verify_installation():
    """Final verification of environment setup."""
    print("\n🧪 Verifying installation...")
    checks = {
        "python": is_command_available("python3"),
        "ros2": is_command_available("ros2"),
        "pip": is_command_available("pip"),
    }

    for tool, available in checks.items():
        print(f"🔹 {tool}: {'✓' if available else '❌'}")

    print("\n✓ Verification completed.")
    print("You can now run:\n   ros2 launch mayuri test_hardware.launch.py")


# ------------------------------------------------------------
# 🚀 Main Installer
# ------------------------------------------------------------

def main():
    print("============================================")
    print("   MAYURI System Installer")
    print("============================================")

    # Load configuration
    config = load_hardware_config()
    device_name = config["device"]["name"]
    gpu_enabled = config["device"].get("gpu_enabled", False)

    print(f"🔧 Target Device: {device_name}")
    print(f"🧠 GPU Acceleration: {'Enabled' if gpu_enabled else 'Disabled'}")

    # Installation steps
    install_python_dependencies()
    install_ros2()
    if gpu_enabled:
        install_cuda_toolkit()
        install_tensor_rt()
    create_directory_structure()
    verify_installation()

    print("\n🎯 MAYURI setup completed successfully!")
    print(f"🕒 Completed at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("👉 Next: Run 'python3 tools/hardware_setup.py' (if not done already).")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⚠️ Installation aborted by user.")
    except Exception as e:
        print(f"\n❌ Error during installation: {e}")
        sys.exit(1)
