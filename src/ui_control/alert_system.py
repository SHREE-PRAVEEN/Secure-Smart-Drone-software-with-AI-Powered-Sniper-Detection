#!/usr/bin/env python3
"""
alert_system.py
-------------------------------------------------
MAYURI Intelligent Alert & Notification System

Monitors system topics (AI detections, failsafe, flight status)
and generates real-time alerts for operators.

==> Real-time monitoring
==> Multi-level alerts (INFO, WARNING, CRITICAL)
==> Audible + visual console alerts
==> Log persistence
==> Modular — integrates with dashboard_ui.py

Author: Shree Praveen(@p_rav_ee_n1082)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
import os
from datetime import datetime
try:
    from playsound import playsound
except ImportError:
    playsound = None


class AlertSystem(Node):
    def __init__(self):
        super().__init__("alert_system")

        # Parameters
        self.declare_parameter("enable_sound", True)
        self.declare_parameter("log_path", "logs/alerts.log")
        self.declare_parameter("sound_critical", "assets/sounds/critical.wav")
        self.declare_parameter("sound_warning", "assets/sounds/warning.wav")
        self.declare_parameter("sound_info", "assets/sounds/info.wav")

        self.enable_sound = bool(self.get_parameter("enable_sound").value)
        self.log_path = str(self.get_parameter("log_path").value)
        self.sound_files = {
            "CRITICAL": str(self.get_parameter("sound_critical").value),
            "WARNING": str(self.get_parameter("sound_warning").value),
            "INFO": str(self.get_parameter("sound_info").value),
        }

        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        self.alert_lock = threading.Lock()

        # Subscriptions
        self.create_subscription(String, "/mayuri/ai/detections", self.on_ai_detections, 10)
        self.create_subscription(String, "/mayuri/failsafe/event", self.on_failsafe_event, 10)
        self.create_subscription(String, "/mayuri/flight/status", self.on_flight_status, 10)

        self.get_logger().info("🔔 AlertSystem initialized and monitoring...")

    # -------------------------------------------------------------
    def play_sound(self, level: str):
        """Play alert sound depending on severity."""
        if not self.enable_sound or not playsound:
            return
        file_path = self.sound_files.get(level)
        if os.path.exists(file_path):
            threading.Thread(target=playsound, args=(file_path,), daemon=True).start()

    def log_alert(self, level: str, source: str, message: str):
        """Log alert to file with timestamp."""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_line = f"[{timestamp}] [{level}] ({source}) {message}\n"
        with self.alert_lock:
            with open(self.log_path, "a") as f:
                f.write(log_line)

    def trigger_alert(self, level: str, source: str, message: str):
        """Unified alert handling."""
        color = {"INFO": "\033[92m", "WARNING": "\033[93m", "CRITICAL": "\033[91m"}.get(level, "")
        reset = "\033[0m"
        print(f"{color}[{level}] {source}: {message}{reset}")

        self.log_alert(level, source, message)
        self.play_sound(level)

    # -------------------------------------------------------------
    def on_ai_detections(self, msg: String):
        """Handle incoming AI detection alerts."""
        try:
            data = json.loads(msg.data)
            detections = data.get("detections", [])
            for det in detections:
                label = det.get("label", "Unknown")
                conf = det.get("confidence", 0.0)
                if conf >= 0.85:
                    self.trigger_alert("CRITICAL", "AI_CORE", f"High-confidence target detected: {label} ({conf*100:.1f}%)")
                elif conf >= 0.6:
                    self.trigger_alert("WARNING", "AI_CORE", f"Possible object: {label} ({conf*100:.1f}%)")
        except Exception as e:
            self.get_logger().warning(f"AI detection parse error: {e}")

    def on_failsafe_event(self, msg: String):
        """Handle system-level failsafe triggers."""
        try:
            event = json.loads(msg.data)
            reason = event.get("reason", "UNKNOWN")
            action = event.get("action", "NONE")
            self.trigger_alert("CRITICAL", "FAILSAFE", f"Triggered: {reason} → Action: {action}")
        except Exception as e:
            self.get_logger().warning(f"Failsafe event error: {e}")

    def on_flight_status(self, msg: String):
        """Monitor for general system warnings (battery, GPS, etc.)."""
        try:
            data = json.loads(msg.data)
            telemetry = data.get("telemetry", {})
            battery = telemetry.get("battery", {}).get("battery_remaining", None)

            if battery is not None:
                if battery <= 20:
                    self.trigger_alert("WARNING", "BATTERY", f"Low battery level: {battery}%")
                elif battery <= 10:
                    self.trigger_alert("CRITICAL", "BATTERY", f"CRITICAL battery level: {battery}%")
        except Exception:
            pass

    # -------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info("🛑 AlertSystem shutting down.")
        super().destroy_node()


# -------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AlertSystem()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
