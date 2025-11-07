"""
hardware_setup.py
------------------------------------------------------------
MAYURI - Hardware Setup & Auto Configuration Utility
Author: MAYURI Robotics Team
Date: 2025-10-22

Description:
    Automatically detects hardware platform, available sensors,
    and system resources. Updates corresponding YAML configuration
    files under config/hardware/.

    Supported Devices:
        - NVIDIA Jetson (Nano, Xavier, Orin)
        - Raspberry Pi (4/5)
        - Intel NUC
        - Desktop/Laptop (RTX GPU or CPU-only)
------------------------------------------------------------
"""

import os
import sys
import yaml
import platform
import subprocess
from pathlib import Path
import psutil
import GPUtil
import socket
from datetime import datetime

# ------------------------------------------------------------
# 🧩 Utility Functions
# ------------------------------------------------------------

def detect_gpu():
    """Detect available GPU (NVIDIA, Intel, or none)."""
    try:
        gpus = GPUtil.getGPUs()
        if not gpus:
            return {"gpu_present": False, "gpu_name": "None"}
        gpu = gpus[0]
        return {
            "gpu_present": True,
            "gpu_name": gpu.name,
            "gpu_memory": f"{gpu.memoryTotal} MB"
        }
    except Exception:
        return {"gpu_present": False, "gpu_name": "None"}

def detect_platform():
    """Identify device type (Jetson, Pi, NUC, etc.)."""
    system = platform.system().lower()
    machine = platform.machine().lower()

    if "aarch64" in machine or "armv7" in machine:
        # ARM-based systems
        if os.path.exists("/usr/sbin/nvpmodel"):
            return "jetson_nano"
        elif Path("/proc/device-tree/model").exists():
            model = Path("/proc/device-tree/model").read_text().lower()
            if "raspberry" in model:
                return "raspberry_pi"
        return "edge_tpu"
    elif "intel" in platform.processor().lower() or "nuc" in platform.node().lower():
        return "intel_nuc"
    else:
        return "desktop"

def detect_network_interfaces():
    """Return list of available network interfaces."""
    try:
        interfaces = psutil.net_if_addrs().keys()
        return list(interfaces)
    except Exception:
        return []

def detect_sensors():
    """Check connected camera or LiDAR devices (basic scan)."""
    sensors = {"camera": False, "lidar": False}
    try:
        # Check for camera
        result = subprocess.run(["v4l2-ctl", "--list-devices"],
                                capture_output=True, text=True)
        sensors["camera"] = "Camera" in result.stdout or "video" in result.stdout
    except FileNotFoundError:
        sensors["camera"] = False

    # Check for LiDAR (via USB)
    try:
        usb_devices = subprocess.check_output("lsusb", shell=True).decode()
        sensors["lidar"] = "RPLIDAR" in usb_devices or "YDLIDAR" in usb_devices
    except Exception:
        sensors["lidar"] = False

    return sensors

# ------------------------------------------------------------
# 🧠 Configuration Generation
# ------------------------------------------------------------

def generate_config(profile_name):
    """Generate and save YAML configuration for detected hardware."""
    print(f"🔍 Detecting hardware for profile: {profile_name}...")

    gpu_info = detect_gpu()
    platform_type = detect_platform()
    interfaces = detect_network_interfaces()
    sensors = detect_sensors()

    config = {
        "device": {
            "name": platform_type,
            "os": platform.system(),
            "gpu_enabled": gpu_info["gpu_present"],
            "gpu_name": gpu_info["gpu_name"],
            "ram_gb": round(psutil.virtual_memory().total / (1024**3), 2),
            "cpu_cores": psutil.cpu_count(logical=True),
            "ip_address": socket.gethostbyname(socket.gethostname())
        },
        "sensors": sensors,
        "network": {
            "interfaces": interfaces,
            "primary": interfaces[0] if interfaces else "unknown"
        },
        "ai": {
            "model": "ai_core/models/yolo_defense.onnx",
            "confidence_threshold": 0.45,
            "hardware_acceleration": "GPU" if gpu_info["gpu_present"] else "CPU"
        },
        "timestamp": datetime.utcnow().isoformat()
    }

    # Save YAML file
    config_dir = Path("config/hardware")
    config_dir.mkdir(parents=True, exist_ok=True)
    output_file = config_dir / f"{profile_name}.yaml"

    with open(output_file, "w") as f:
        yaml.dump(config, f, sort_keys=False)

    print(f"✓ Configuration saved: {output_file}")
    print("📡 Detected:")
    print(f"   • Device Type: {platform_type}")
    print(f"   • GPU: {gpu_info['gpu_name']}")
    print(f"   • Camera: {'✓' if sensors['camera'] else '❌'}")
    print(f"   • LiDAR: {'✓' if sensors['lidar'] else '❌'}")
    print(f"   • Network: {interfaces}")
    return output_file

# ------------------------------------------------------------
# 🚀 Entry Point
# ------------------------------------------------------------

if __name__ == "__main__":
    print("============================================")
    print("   MAYURI Hardware Setup Utility")
    print("============================================")

    import argparse
    parser = argparse.ArgumentParser(description="MAYURI Hardware Setup Tool")
    parser.add_argument("--profile", type=str, default="auto_detected",
                        help="Profile name for the hardware config file (e.g., jetson_nano, desktop, nuc)")
    args = parser.parse_args()

    try:
        config_path = generate_config(args.profile)
        print(f"\n🚀 Hardware setup completed successfully!\nConfiguration file: {config_path}")
        print("Run 'ros2 launch mayuri test_hardware.launch.py' to verify setup.")
    except Exception as e:
        print(f"❌ Error during setup: {e}")
        sys.exit(1)
