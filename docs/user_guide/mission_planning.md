# 🧭 MAYURI – Mission Planning Guide

**Version:** 1.0  
**Last Updated:** 2025-10-22  
**Author:** MAYURI Robotics Team  

---

## 🛰️ Overview

The **Mission Planner** is the **central control module** of MAYURI, responsible for defining, scheduling, and executing autonomous missions such as:

- Border surveillance  
- Patrol missions  
- Rescue operations  
- Threat tracking or monitoring  

It orchestrates all active subsystems:
> **AI Core + Navigation + Communication + UI + System Diagnostics**

and ensures synchronization across all components in real time.

---

## 🧩 Core Module

| File | Location | Description |
|------|-----------|-------------|
| `mission_planner.py` | `src/core/mission_planner.py` | Main orchestration node for missions |
| `patrol_mission.yaml` | `config/missions/` | Defines path and AI behaviors for patrol |
| `rescue_operation.yaml` | `config/missions/` | Rescue-focused mission definition |
| `threat_tracking.yaml` | `config/missions/` | Target tracking mission setup |
| `border_surveillance.yaml` | `config/missions/` | Continuous perimeter monitoring |

---

## ⚙️ Mission Planner Responsibilities

| Function | Description |
|-----------|--------------|
| **Mission Loading** | Loads YAML mission configuration |
| **Subsystem Coordination** | Initializes AI, sensors, and navigation modules |
| **Event Scheduling** | Manages time-based triggers and checkpoints |
| **Command Dispatch** | Sends ROS2 messages to start/stop nodes |
| **Health Monitoring** | Verifies system readiness before launch |
| **Data Logging** | Records telemetry and AI detections for analysis |

---

## 🧾 Mission Configuration (YAML)

Each mission is defined as a YAML file in `config/missions/`.  
Here’s a template:

```yaml
# Example: patrol_mission.yaml
mission:
  id: patrol_001
  name: Border Patrol Alpha
  mode: autonomous
  start_time: 2025-10-22T06:30:00Z
  duration: 45m

waypoints:
  - { lat: 28.6139, lon: 77.2090, alt: 150, action: "scan" }
  - { lat: 28.6145, lon: 77.2102, alt: 160, action: "hover" }
  - { lat: 28.6150, lon: 77.2115, alt: 140, action: "return" }

ai:
  model: ai_core/models/yolo_defense.onnx
  confidence_threshold: 0.45
  tracking: true
  record_feed: true

communication:
  link: "secure_mesh"
  encryption: "AES-256-GCM"

failsafe:
  low_battery: "return_home"
  comm_loss: "hover"
  overheat: "land"
