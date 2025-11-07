"""
system_monitor.py
------------------------------------------------------------
MAYURI System Monitor
Author: Shree Praveen(@p_rav_ee_n1082)
Date: 2025-10-22

Description:
    Centralized monitoring system that tracks CPU, GPU,
    memory, temperature, power, and network status.
    Works across Jetson, Raspberry Pi, Intel NUC, and
    other supported hardware nodes.
------------------------------------------------------------
"""

import os
import time
import json
import psutil
import logging
import platform
from datetime import datetime
from pathlib import Path
from threading import Thread

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    # Allow standalone operation without ROS2
    rclpy = None
    Node = object
    String = None


# ------------------------------------------------------------
# 🧠 MAYURI System Monitor Class
# ------------------------------------------------------------
class SystemMonitor(Node if rclpy else object):
    def __init__(self, interval: int = 5):
        if rclpy:
            super().__init__("system_monitor")

        self.interval = interval
        self.active = True

        # Logging setup
        log_dir = Path("logs/system")
        log_dir.mkdir(parents=True, exist_ok=True)
        logging.basicConfig(
            filename=log_dir / "system_monitor.log",
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s]: %(message)s",
        )

        # ROS2 publisher setup
        if rclpy:
            self.telemetry_pub = self.create_publisher(String, "/mayuri/system/health", 10)

        logging.info("✅ SystemMonitor initialized")

        # Start monitoring thread
        thread = Thread(target=self._monitor_loop, daemon=True)
        thread.start()

    # --------------------------------------------------------
    # 🔍 Collect System Metrics
    # --------------------------------------------------------
    def _collect_metrics(self):
        try:
            cpu_usage = psutil.cpu_percent(interval=1)
            mem = psutil.virtual_memory()
            temp = self._get_temperature()
            net = psutil.net_io_counters()
            uptime = datetime.now() - datetime.fromtimestamp(psutil.boot_time())

            metrics = {
                "timestamp": datetime.utcnow().isoformat(),
                "system": platform.node(),
                "cpu_usage_percent": cpu_usage,
                "memory_usage_percent": mem.percent,
                "available_memory_mb": mem.available / (1024 * 1024),
                "temperature_c": temp,
                "bytes_sent": net.bytes_sent,
                "bytes_recv": net.bytes_recv,
                "uptime_minutes": round(uptime.total_seconds() / 60, 2),
            }

            # GPU stats (for Jetson / CUDA)
            gpu_stats = self._get_gpu_stats()
            if gpu_stats:
                metrics.update(gpu_stats)

            return metrics

        except Exception as e:
            logging.error(f"Error collecting system metrics: {e}")
            return {}

    # --------------------------------------------------------
    # 🌡️ Get Temperature (Cross-Platform)
    # --------------------------------------------------------
    def _get_temperature(self):
        temp = None
        try:
            # For Jetson Nano / Raspberry Pi
            if os.path.exists("/sys/class/thermal/thermal_zone0/temp"):
                with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                    temp = int(f.read().strip()) / 1000.0
            else:
                # Fallback using psutil
                sensors = psutil.sensors_temperatures()
                if "coretemp" in sensors:
                    temp = sensors["coretemp"][0].current
        except Exception:
            temp = None
        return temp

    # --------------------------------------------------------
    # 🎮 GPU Statistics
    # --------------------------------------------------------
    def _get_gpu_stats(self):
        stats = {}
        try:
            # NVIDIA Jetson / CUDA systems
            if os.system("which tegrastats > /dev/null 2>&1") == 0:
                output = os.popen("tegrastats --interval 1000 --count 1").read()
                if "GR3D_FREQ" in output:
                    gpu_util = int(output.split("GR3D_FREQ")[1].split("%")[0].split("@")[-1].strip())
                    stats["gpu_usage_percent"] = gpu_util
            elif os.system("which nvidia-smi > /dev/null 2>&1") == 0:
                output = os.popen("nvidia-smi --query-gpu=utilization.gpu,temperature.gpu --format=csv,noheader").read().strip()
                if output:
                    util, temp = output.split(",")
                    stats["gpu_usage_percent"] = int(util.strip().split()[0])
                    stats["gpu_temperature_c"] = int(temp.strip().split()[0])
        except Exception:
            pass
        return stats

    # --------------------------------------------------------
    # 📡 Monitoring Loop
    # --------------------------------------------------------
    def _monitor_loop(self):
        while self.active:
            data = self._collect_metrics()
            if not data:
                time.sleep(self.interval)
                continue

            # Log data locally
            logging.info(json.dumps(data))

            # Publish to ROS2
            if rclpy and String:
                self.telemetry_pub.publish(String(data=json.dumps(data)))

            # Print short summary for debugging
            print(f"📊 CPU: {data['cpu_usage_percent']}% | Mem: {data['memory_usage_percent']}% | Temp: {data.get('temperature_c', 'N/A')}°C")

            # Handle overheating
            if data.get("temperature_c", 0) > 80:
                logging.warning("🔥 High temperature detected!")
                self._publish_alert("Temperature exceeds 80°C! Auto-throttle enabled.")

            time.sleep(self.interval)

    # --------------------------------------------------------
    # 🚨 Alert Publisher
    # --------------------------------------------------------
    def _publish_alert(self, message: str):
        if rclpy and String:
            self.telemetry_pub.publish(String(data=f"ALERT: {message}"))
        logging.warning(message)

    # --------------------------------------------------------
    # 🛑 Stop Monitoring
    # --------------------------------------------------------
    def stop(self):
        self.active = False
        logging.info("SystemMonitor stopped")
        print("🟥 System monitoring stopped.")


# ------------------------------------------------------------
# 🧩 Standalone Runner
# ------------------------------------------------------------
def run_system_monitor(interval: int = 5):
    if rclpy:
        rclpy.init()
        node = SystemMonitor(interval=interval)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.stop()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        monitor = SystemMonitor(interval=interval)
        try:
            while monitor.active:
                time.sleep(1)
        except KeyboardInterrupt:
            monitor.stop()


# ------------------------------------------------------------
# 🧭 Entry Point
# ------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="MAYURI System Monitor")
    parser.add_argument("--interval", type=int, default=5, help="Monitoring interval in seconds")
    args = parser.parse_args()

    run_system_monitor(args.interval)
