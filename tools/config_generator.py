"""
config_generator.py
------------------------------------------------------------
MAYURI - Interactive Configuration Generator
Author: MAYURI Robotics Team
Date: 2025-10-22

Description:
    Command-line tool to create and modify YAML configuration
    files for MAYURI’s hardware, missions, and security settings.

Supported Config Types:
    - hardware
    - mission
    - security
------------------------------------------------------------
"""

import os
import sys
import yaml
from pathlib import Path
from datetime import datetime

# ------------------------------------------------------------
# 🧩 Helper Functions
# ------------------------------------------------------------

def save_yaml(file_path, data):
    """Save YAML data safely to the given file path."""
    Path(file_path).parent.mkdir(parents=True, exist_ok=True)
    with open(file_path, "w") as f:
        yaml.dump(data, f, sort_keys=False)
    print(f"✓ Configuration saved at: {file_path}\n")


def ask(prompt, default=None, choices=None):
    """Utility to ask for user input with optional default or choices."""
    if choices:
        prompt += f" ({'/'.join(choices)})"
    if default:
        prompt += f" [default: {default}]"
    prompt += ": "
    val = input(prompt).strip()
    if not val and default is not None:
        return default
    if choices and val not in choices:
        print(f"⚠️ Invalid choice. Please choose from {choices}.")
        return ask(prompt, default, choices)
    return val


# ------------------------------------------------------------
# ⚙️ Hardware Configuration Generator
# ------------------------------------------------------------

def generate_hardware_config():
    print("\n🧠 Generating Hardware Configuration...")
    device_name = ask("Device name (e.g. jetson_nano, raspberry_pi, desktop)", "desktop")
    gpu_enabled = ask("Enable GPU acceleration?", "yes", ["yes", "no"]) == "yes"
    gpu_name = ask("GPU model name", "NVIDIA GeForce RTX 2050" if gpu_enabled else "None")
    ram_gb = float(ask("RAM (in GB)", "16"))
    cpu_cores = int(ask("CPU cores", "8"))
    ai_model = ask("Default AI model path", "ai_core/models/yolo_defense.onnx")

    config = {
        "device": {
            "name": device_name,
            "os": os.uname().sysname,
            "gpu_enabled": gpu_enabled,
            "gpu_name": gpu_name,
            "ram_gb": ram_gb,
            "cpu_cores": cpu_cores,
        },
        "ai": {
            "model": ai_model,
            "confidence_threshold": 0.45,
            "hardware_acceleration": "GPU" if gpu_enabled else "CPU"
        },
        "timestamp": datetime.utcnow().isoformat()
    }

    filename = f"config/hardware/{device_name}.yaml"
    save_yaml(filename, config)


# ------------------------------------------------------------
# 🧭 Mission Configuration Generator
# ------------------------------------------------------------

def generate_mission_config():
    print("\n🛰️ Generating Mission Configuration...")
    mission_name = ask("Mission name", "patrol_mission")
    mode = ask("Mode (autonomous / semi-autonomous / manual)", "autonomous")
    speed = float(ask("Flight speed (m/s)", "3.5"))
    altitude_mode = ask("Altitude mode (AGL / MSL)", "AGL")
    return_home = ask("Return to home after mission?", "yes", ["yes", "no"]) == "yes"

    waypoints = []
    print("\n📍 Enter waypoint coordinates (lat, lon, alt). Type 'done' when finished.")
    while True:
        lat = ask("Latitude (or 'done' to finish)")
        if lat.lower() == "done":
            break
        lon = ask("Longitude")
        alt = ask("Altitude (meters)")
        waypoints.append({"lat": float(lat), "lon": float(lon), "altitude": float(alt)})

    config = {
        "mission": {
            "name": mission_name,
            "mode": mode,
            "start_time": datetime.utcnow().isoformat(),
            "waypoints": waypoints
        },
        "parameters": {
            "speed": speed,
            "altitude_mode": altitude_mode,
            "return_home": return_home
        },
        "ai": {
            "target_detection": True,
            "object_classification": True,
            "thermal_scanning": False,
            "data_recording": True
        },
        "failsafe": {
            "battery_threshold": 15,
            "gps_loss_action": "hover",
            "comm_loss_action": "return_home"
        }
    }

    filename = f"config/missions/{mission_name}.yaml"
    save_yaml(filename, config)


# ------------------------------------------------------------
# 🔒 Security Configuration Generator
# ------------------------------------------------------------

def generate_security_config():
    print("\n🔐 Generating Security Configuration...")

    comm_protocol = ask("Communication protocol (mesh / wifi / ethernet)", "mesh")
    encryption_level = ask("Encryption level (AES-128 / AES-256)", "AES-256")
    key_rotation = ask("Key rotation interval (minutes)", "60")
    node_auth = ask("Enable node authentication?", "yes", ["yes", "no"]) == "yes"

    config = {
        "security": {
            "protocol": comm_protocol,
            "encryption": encryption_level,
            "key_rotation_minutes": int(key_rotation),
            "authentication_enabled": node_auth,
            "last_updated": datetime.utcnow().isoformat()
        },
        "permissions": {
            "authorized_nodes": [
                {"id": "MAYURI-AI-CORE", "role": "detector"},
                {"id": "MAYURI-NAV", "role": "navigator"},
                {"id": "MAYURI-UI", "role": "interface"}
            ]
        }
    }

    filename = "config/security/comm_protocols.yaml"
    save_yaml(filename, config)


# ------------------------------------------------------------
# 🚀 Entry Point
# ------------------------------------------------------------

def main():
    print("============================================")
    print("   MAYURI Configuration Generator")
    print("============================================")
    print("Supported config types: hardware | mission | security")

    config_type = ask("Which configuration do you want to create?", choices=["hardware", "mission", "security"])

    if config_type == "hardware":
        generate_hardware_config()
    elif config_type == "mission":
        generate_mission_config()
    elif config_type == "security":
        generate_security_config()
    else:
        print("❌ Invalid selection.")
        sys.exit(1)

    print("🎯 Configuration generation complete.")
    print("You can now run `ros2 launch mayuri full_system.launch.py` to use your new configuration.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⚠️ Process interrupted by user.")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        sys.exit(1)
