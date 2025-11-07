    # ⚙️ MAYURI – Hardware Requirements

**Version:** 1.0  
**Last Updated:** 2025-10-22  
**Author:** MAYURI Robotics Team  

---

## 🛰️ Overview

MAYURI is designed to run efficiently across a range of edge and desktop computing platforms — from low-power single-board computers (SBCs) to high-performance AI edge systems.

The hardware configuration determines how many subsystems (AI, Navigation, Communication, UI) can run simultaneously in real time.

---

## ⚙️ System Performance Levels

| Mode | Description | Use Case |
|------|--------------|----------|
| 🧩 **Development Mode** | Runs on laptops/desktops with GPU acceleration | Local testing, AI training, debugging |
| 🚁 **Edge Deployment Mode** | Runs on embedded systems (Jetson, Pi, NUC) | Field operations, mission deployment |
| ☁️ **Simulation Mode** | Runs without sensors or hardware | Virtual testing, data replay, ML inference validation |

---

## 💻 Minimum Hardware Requirements

| Component | Specification | Notes |
|------------|----------------|-------|
| **CPU** | Intel i5 / AMD Ryzen 5 (or equivalent ARM A72) | Quad-core minimum |
| **GPU (Optional)** | NVIDIA GeForce 2050 / Jetson Nano GPU / Intel Iris Xe | Required for real-time AI inference |
| **RAM** | 8 GB | 16 GB recommended for full AI stack |
| **Storage** | 64 GB SSD or higher | SSD preferred for faster model loading |
| **OS** | Ubuntu 22.04 LTS (recommended) | ROS2 Humble or newer |
| **Python** | 3.9 – 3.11 | Matches ROS2 compatibility |
| **ROS2** | Humble / Iron / Jazzy | Core middleware for MAYURI nodes |

> ✅ Your Asus Vivobook (i5 + RTX 2050) **fully supports MAYURI development** — capable of running the **complete AI + UI stack** locally.

---

## ⚙️ Recommended Development Setup (Laptop / PC)

| Component | Specification |
|------------|----------------|
| **CPU** | Intel i7 (11th Gen or later) / Ryzen 7 |
| **GPU** | NVIDIA RTX 2050 or higher (4GB+ VRAM) |
| **RAM** | 16 GB DDR4 or DDR5 |
| **Storage** | 512 GB NVMe SSD |
| **Display** | 1080p / 1440p for visualization |
| **OS** | Ubuntu 22.04 LTS / Windows Subsystem for Linux 2 (WSL2) |
| **Network** | Wi-Fi 6 / Ethernet |
| **Ports** | 2× USB 3.0, 1× USB-C, HDMI |

**Performance Notes:**
- YOLOv8 inference: ~25–30 FPS on RTX 2050  
- Sensor Fusion + UI: <15% CPU usage  
- ROS2 launch startup: ~6 seconds  

---

## 🧠 Edge Deployment Setup (Jetson Nano / Xavier / NUC)

| Component | Configuration | Notes |
|------------|----------------|------|
| **Jetson Nano** | 4GB RAM, Ubuntu 20.04, JetPack 5.1 | Light AI workloads (YOLOv8n, thermal fusion) |
| **Jetson Xavier NX** | 8GB RAM, Ubuntu 22.04, JetPack 6 | Medium to heavy AI workloads |
| **Intel NUC 11** | i7 CPU, 16GB RAM, Ubuntu 22.04 | Best balance for field testing |
| **Edge TPU** | Coral USB / M.2 Accelerator | Optional AI offloading for low-power missions |
| **Storage** | 128GB SSD / microSD | Store logs, models, and video frames |
| **Power** | 5V–12V DC (UPS backed) | Stable supply required during long missions |

---

## 📡 Sensor Requirements

| Sensor | Example Model | Interface | Function |
|---------|----------------|------------|-----------|
| **RGB Camera** | Logitech C920 / Arducam | USB / MIPI | Visual feed for YOLO |
| **Thermal Camera** | FLIR Lepton 3.5 | SPI / I2C | Infrared heat signatures |
| **LiDAR** | RPLIDAR A1 / A2 | Serial / USB | Depth and obstacle detection |
| **Radar** | Acconeer XM132 / Navtech Radar | SPI / CAN | Short-range motion awareness |
| **GPS + IMU** | u-blox NEO-M8N | UART / I2C | Localization and orientation |
| **Microphone (optional)** | USB or MEMS | Sound event awareness |

---

## 🔒 Communication Hardware

| Component | Example | Function |
|------------|----------|----------|
| **Mesh Radio** | Digi XBee / ESP-NOW | Node-to-node telemetry |
| **Wi-Fi Module** | Intel AX200 / ESP32 | Command and data streaming |
| **Ethernet** | Gigabit adapter | Base station connection |
| **Antenna** | 2.4GHz / 5GHz dual-band | Range and reliability |

> All communication modules use **AES-256 encryption** via the `encryption_manager.py` and `secure_comms.py` subsystems.

---

## 🧭 Peripheral Hardware

| Component | Purpose |
|------------|----------|
| **OLED/Touch Display (optional)** | Local mission display |
| **Cooling Fan** | Prevents GPU thermal throttling |
| **UPS / Battery** | Safe shutdown and power stability |
| **GPS Module** | Geo-localization |
| **LED Indicators** | System status visualization |

---

## 🧩 ROS2 Hardware Interfaces

| Interface | Node | Purpose |
|------------|------|----------|
| `/mayuri/sensor/camera` | camera_manager.py | RGB/thermal image feed |
| `/mayuri/sensor/lidar` | lidar_processor.py | 2D/3D LiDAR scans |
| `/mayuri/sensor/radar` | radar_processor.py | Motion detection |
| `/mayuri/navigation/odom` | flight_controller.py | Odometry and path correction |
| `/mayuri/system/health` | system_monitor.py | System temperature, CPU, GPU |

---

## ⚡ Power Consumption Estimates

| Platform | Idle | Active AI | Full Stack |
|-----------|------|------------|-------------|
| Jetson Nano | 5W | 9W | 12W |
| Raspberry Pi 5 | 6W | 10W | 13W |
| Intel NUC | 8W | 15W | 22W |
| Laptop (RTX 2050) | 10W | 40W | 55W |

---

## 🧮 Summary

- ✅ **Your RTX 2050 laptop** = fully capable of development and testing.  
- 🛰️ **Jetson / NUC** recommended for real deployments.  
- ⚙️ **Minimum 8 GB RAM, 4-core CPU, GPU/TPU acceleration** preferred.  
- 🔋 **Stable power & cooling** are essential for long missions.  

---

## 📎 Related Configuration Files

| File | Purpose |
|------|----------|
| `config/hardware/jetson_nano.yaml` | Jetson-specific tuning |
| `config/hardware/raspberry_pi.yaml` | Pi setup (TFLite optimized) |
| `config/hardware/intel_nuc.yaml` | Desktop deployment setup |
| `config/hardware/edge_tpu.yaml` | TPU inference optimization |

---

> 💡 **Tip:** For best results, use `docker/Dockerfile.jetson` or `docker/Dockerfile.dev` to create consistent environments across devices.

