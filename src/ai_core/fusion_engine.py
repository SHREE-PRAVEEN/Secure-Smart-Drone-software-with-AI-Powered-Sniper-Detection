"""
fusion_engine.py
------------------------------------------------------------
MAYURI Multi-Sensor Fusion Engine
Author: Shree Praveen(@p_rav_ee_n1082)
Date: 2025-10-22

Description:
    Fuses data from RGB camera, thermal, radar, and lidar
    sources into a unified situational map for downstream
    AI modules (classification, navigation, diagnostics).
    Supports both standalone and ROS2 operation.
------------------------------------------------------------
"""

import cv2
import numpy as np
import json
import time
import logging
from pathlib import Path
from datetime import datetime
from threading import Lock

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge
except ImportError:
    rclpy = None
    Node = object
    CvBridge = None
    Image = None
    String = None


# ------------------------------------------------------------
# 🧩 Fusion Engine
# ------------------------------------------------------------
class FusionEngine(Node if rclpy else object):
    def __init__(self):
        if rclpy:
            super().__init__("fusion_engine")

        self.lock = Lock()
        self.bridge = CvBridge() if CvBridge else None

        # Latest sensor frames
        self.rgb_frame = None
        self.thermal_frame = None
        self.radar_data = None
        self.lidar_data = None

        # Logging setup
        log_dir = Path("logs/ai")
        log_dir.mkdir(parents=True, exist_ok=True)
        logging.basicConfig(
            filename=log_dir / "fusion_engine.log",
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s]: %(message)s",
        )

        # ROS2 Topics
        if rclpy:
            self.rgb_sub = self.create_subscription(Image, "/mayuri/sensors/rgb", self._rgb_callback, 10)
            self.thermal_sub = self.create_subscription(Image, "/mayuri/sensors/thermal", self._thermal_callback, 10)
            self.radar_sub = self.create_subscription(String, "/mayuri/sensors/radar", self._radar_callback, 10)
            self.lidar_sub = self.create_subscription(String, "/mayuri/sensors/lidar", self._lidar_callback, 10)
            self.fusion_pub = self.create_publisher(String, "/mayuri/ai/fusion_output", 10)

        logging.info("✅ FusionEngine initialized")

    # --------------------------------------------------------
    # 🖼️ Sensor Callbacks
    # --------------------------------------------------------
    def _rgb_callback(self, msg):
        if self.bridge:
            self.rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _thermal_callback(self, msg):
        if self.bridge:
            self.thermal_frame = self.bridge.imgmsg_to_cv2(msg, "mono8")

    def _radar_callback(self, msg):
        try:
            self.radar_data = json.loads(msg.data)
        except Exception:
            self.radar_data = None

    def _lidar_callback(self, msg):
        try:
            self.lidar_data = json.loads(msg.data)
        except Exception:
            self.lidar_data = None

    # --------------------------------------------------------
    # 🧠 Fusion Pipeline
    # --------------------------------------------------------
    def fuse(self):
        with self.lock:
            if self.rgb_frame is None:
                return None

            fusion_result = {
                "timestamp": datetime.utcnow().isoformat(),
                "objects": [],
                "environment": {},
            }

            # 1️⃣ Start with RGB frame
            fused_frame = self.rgb_frame.copy()

            # 2️⃣ Blend with thermal overlay (if available)
            if self.thermal_frame is not None:
                thermal_resized = cv2.resize(self.thermal_frame, (fused_frame.shape[1], fused_frame.shape[0]))
                thermal_colored = cv2.applyColorMap(thermal_resized, cv2.COLORMAP_JET)
                fused_frame = cv2.addWeighted(fused_frame, 0.7, thermal_colored, 0.3, 0)

            # 3️⃣ Radar data overlay
            if self.radar_data:
                for obj in self.radar_data.get("targets", []):
                    x, y = int(obj["x"]), int(obj["y"])
                    strength = obj.get("strength", 1.0)
                    cv2.circle(fused_frame, (x, y), 6, (0, 255, 255), -1)
                    fusion_result["objects"].append({"source": "radar", "x": x, "y": y, "strength": strength})

            # 4️⃣ Lidar distance overlay
            if self.lidar_data:
                avg_dist = np.mean([p["distance"] for p in self.lidar_data.get("points", [])]) if self.lidar_data.get("points") else None
                fusion_result["environment"]["avg_distance_m"] = round(float(avg_dist), 2) if avg_dist else None

            # 5️⃣ Temperature statistics (thermal)
            if self.thermal_frame is not None:
                mean_temp = np.mean(self.thermal_frame)
                fusion_result["environment"]["mean_thermal_intensity"] = round(float(mean_temp), 2)

            # Publish result
            self._publish_result(fusion_result)

            return fused_frame, fusion_result

    # --------------------------------------------------------
    # 📡 Publish Fusion Output
    # --------------------------------------------------------
    def _publish_result(self, result):
        if rclpy and String:
            msg = String()
            msg.data = json.dumps(result)
            self.fusion_pub.publish(msg)
        logging.info(f"Fusion output: {json.dumps(result)}")

    # --------------------------------------------------------
    # 🧩 Visualization (for Debugging)
    # --------------------------------------------------------
    def visualize(self):
        print("🎥 Starting Fusion Visualization... Press 'q' to quit.")
        while True:
            fused = self.fuse()
            if fused is None:
                continue

            frame, info = fused
            cv2.putText(frame, "MAYURI Fusion View", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow("MAYURI - Fusion Engine", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cv2.destroyAllWindows()

    # --------------------------------------------------------
    # 🛑 Stop Node
    # --------------------------------------------------------
    def stop(self):
        print("🟥 Stopping Fusion Engine...")
        self.active = False
        if rclpy:
            self.destroy_node()


# ------------------------------------------------------------
# 🧩 Standalone Runner
# ------------------------------------------------------------
def run_fusion_engine():
    if rclpy:
        rclpy.init()
        node = FusionEngine()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.stop()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        fusion = FusionEngine()
        fusion.visualize()


# ------------------------------------------------------------
# 🧭 Entry Point
# ------------------------------------------------------------
if __name__ == "__main__":
    run_fusion_engine()
