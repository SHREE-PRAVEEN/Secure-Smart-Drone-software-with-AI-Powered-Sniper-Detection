# 🧠 MAYURI – System Architecture Overview

**Version:** 1.0  
**Last Updated:** 2025-10-22  
**Author:** MAYURI Robotics Team  

---

## 🛰️ Overview

**MAYURI** (*Modular Autonomous Yielding Unmanned Robotic Intelligence*)  
is a **multi-sensor AI-driven autonomous robotics platform** designed for surveillance, environmental awareness, and intelligent decision-making.

It is built with a **modular, ROS2-based microservice architecture** that allows each subsystem (AI, navigation, communication, sensors, UI, and security) to operate **independently yet cooperatively**, ensuring real-time adaptability, fault tolerance, and scalability.

---

## 🧩 Core Architecture Layers

### 1. **Hardware Layer**
Physical computing and sensor ecosystem that powers MAYURI.

| Component | Example | Role |
|------------|----------|------|
| Compute Unit | Jetson Nano, Raspberry Pi 5, Intel NUC | AI computation, inference, and mission control |
| Sensors | RGB, Thermal, LiDAR, Radar, GPS, IMU | Perception and environment mapping |
| Communication | Wi-Fi, Mesh Radio, Ethernet | Secure multi-node communication |
| Power & Safety | Smart battery, fan, cooling | Power management, safety failsafes |

**Hardware Configurations:**  
- Defined in `config/hardware/*.yaml`  
- Enables platform-aware optimizations (Jetson, Pi, NUC, Edge TPU)

---

### 2. **System Core Layer**

Located in: `src/core/`

This layer handles **mission logic, system monitoring, and diagnostics.**

| Module | Function |
|---------|-----------|
| `mission_planner.py` | Loads mission YAMLs, orchestrates AI modules |
| `system_monitor.py` | Tracks CPU, GPU, memory, network health |
| `diagnostics.py` | Predicts anomalies, generates health reports |
| `failsafe_manager.py` | Manages emergency stop, power cut, and recovery routines |

The **Core Layer** acts as MAYURI’s **brainstem**, keeping the robot operational, safe, and adaptive.

---

### 3. **AI Perception Layer**

Located in: `src/ai_core/`

Handles real-time AI vision, classification, and sensor fusion.

| Module | Function |
|---------|-----------|
| `target_detection.py` | YOLO/ONNX object detection engine |
| `classification.py` | Classifies detected objects (vehicle, human, etc.) |
| `motion_tracking.py` | Tracks movement across frames |
| `fusion_engine.py` | Combines RGB, IR, LiDAR, and radar data |
| `thermal_detector.py` | Detects heat signatures in thermal imagery |

The **AI Core** produces a live stream of perception data, enabling awareness even in low-light or obstructed environments.

---

### 4. **Navigation & Control Layer**

Located in: `src/navigation/`

Responsible for autonomous movement, path optimization, and obstacle handling.

| Module | Function |
|---------|-----------|
| `flight_controller.py` | Interface with autopilot or drone flight stack |
| `autopilot_manager.py` | Maintains stability and speed control |
| `path_planner.py` | Generates safe and optimized routes |
| `obstacle_avoidance.py` | Prevents collisions in dynamic environments |
| `failsafe_manager.py` | Emergency safety operations |

The navigation layer ensures **smooth motion, adaptive rerouting**, and **hardware feedback integration**.

---

### 5. **Sensor Processing Layer**

Located in: `src/sensors/`

Handles multi-modal sensor input streams.

| Module | Function |
|---------|-----------|
| `camera_manager.py` | RGB & thermal camera management |
| `lidar_processor.py` | Point cloud and depth mapping |
| `radar_processor.py` | Short-range motion sensing |
| `sensor_fusion.py` | Combines sensor inputs into unified data |
| `calibration.py` | Auto-calibrates and synchronizes sensors |

---

### 6. **Communication & Security Layer**

Located in: `src/communication/`

Ensures secure, encrypted communication and networking between MAYURI nodes.

| Module | Function |
|---------|-----------|
| `secure_comms.py` | AES-256 encrypted telemetry exchange |
| `encryption_manager.py` | Key management and protocol handling |
| `mesh_network.py` | Handles ad-hoc multi-node networking |
| `command_bridge.py` | Connects AI core with external control centers |

Security configuration and protocols are stored in:  
`config/security/comm_protocols.yaml` and `config/security/aes_keys.json`.

---

### 7. **User Interface Layer**

Located in: `src/ui_control/`

Provides visual feedback and manual override.

| Module | Function |
|---------|-----------|
| `dashboard_ui.py` | Real-time mission dashboard |
| `map_visualizer.py` | Displays live position and tracking |
| `alert_system.py` | Issues audio/visual alerts for system events |

The UI communicates through ROS2 topics like `/mayuri/mission/status` and `/mayuri/system/health`.

---

### 8. **Configuration & Launch Layer**

Located in:  
- `config/` → hardware, mission, and security YAMLs  
- `launch/` → ROS2 launch scripts for modular startup  

| Launch File | Purpose |
|--------------|----------|
| `ai_only.launch.py` | Launches only AI subsystems for testing |
| `full_system.launch.py` | Starts all modules (AI, nav, comm, UI) |
| `simulation.launch.py` | Runs system in virtual mode |
| `test_hardware.launch.py` | Performs hardware diagnostics |

---

### 9. **Docker & Deployment Layer**

Located in: `docker/`

| File | Description |
|-------|--------------|
| `Dockerfile.dev` | Developer environment with ROS2 + AI dependencies |
| `Dockerfile.jetson` | Optimized runtime image for NVIDIA Jetson |
| `docker-compose.yml` | Multi-container deployment configuration |

This layer ensures MAYURI can be deployed easily across cloud, edge, and local hardware setups.

---

## 🔗 Data Flow Overview

```text
[ Sensors ]
   ↓
Sensor Layer → AI Core → Fusion Engine → Motion Tracking
   ↓                            ↓
Navigation Layer ← Mission Planner ← System Core
   ↓                            ↓
Communication Layer → Secure Mesh Network
   ↓
User Interface Layer → Dashboard & Alerts
