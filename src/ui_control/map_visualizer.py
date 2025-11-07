#!/usr/bin/env python3
"""
map_visualizer.py
-------------------------------------------------
MAYURI Map Visualization System

Displays real-time drone location, mission waypoints, and detected targets on an interactive map.
Integrates with flight_controller, mission_planner, and ai_core modules.

==>Live GPS tracking
==> Mission route visualization
==> Threat / object markers from AI core
==> Works in simulation or real flight mode
==> Exports HTML map snapshots
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import folium
from folium.plugins import MarkerCluster
import os
import time
import threading
from typing import Dict, List, Tuple

# Default map output path
MAP_PATH = "docs/user_guide/live_map.html"


class MapVisualizer(Node):
    def __init__(self):
        super().__init__("map_visualizer")

        # Parameters
        self.declare_parameter("update_rate", 1.0)  # Hz
        self.declare_parameter("map_center", [12.9716, 77.5946])  # Default Bangalore coords
        self.declare_parameter("zoom_start", 14)
        self.declare_parameter("auto_open", False)
        self.declare_parameter("output_path", MAP_PATH)

        self.update_rate = float(self.get_parameter("update_rate").value)
        self.map_center = list(self.get_parameter("map_center").value)
        self.zoom_start = int(self.get_parameter("zoom_start").value)
        self.auto_open = bool(self.get_parameter("auto_open").value)
        self.output_path = str(self.get_parameter("output_path").value)

        # Internal state
        self.drone_position: Dict[str, float] = {"lat": None, "lon": None, "alt": None}
        self.mission_points: List[Tuple[float, float]] = []
        self.detected_objects: List[Dict] = []
        self.map_lock = threading.Lock()

        # Subscriptions
        self.create_subscription(String, "/mayuri/flight/status", self.on_flight_status, 10)
        self.create_subscription(String, "/mayuri/mission/waypoints", self.on_mission_waypoints, 10)
        self.create_subscription(String, "/mayuri/ai/detections", self.on_ai_detections, 10)

        # Start map update thread
        self.running = True
        self.map_thread = threading.Thread(target=self.map_loop, daemon=True)
        self.map_thread.start()

        self.get_logger().info("🗺️ MapVisualizer initialized and running.")

    # ---------------------------------------------------------------------
    def on_flight_status(self, msg: String):
        """Update drone GPS position from flight controller."""
        try:
            data = json.loads(msg.data)
            pos = data.get("position", {})
            if pos.get("lat") and pos.get("lon"):
                self.drone_position = {
                    "lat": pos["lat"],
                    "lon": pos["lon"],
                    "alt": pos.get("alt", 0.0)
                }
        except Exception as e:
            self.get_logger().warning(f"Failed to parse flight status: {e}")

    def on_mission_waypoints(self, msg: String):
        """Update list of mission waypoints."""
        try:
            data = json.loads(msg.data)
            self.mission_points = [(p["lat"], p["lon"]) for p in data.get("waypoints", [])]
        except Exception as e:
            self.get_logger().warning(f"Waypoint parse error: {e}")

    def on_ai_detections(self, msg: String):
        """Update AI-based detected object locations."""
        try:
            data = json.loads(msg.data)
            detections = data.get("detections", [])
            self.detected_objects = [
                {
                    "lat": d.get("lat"),
                    "lon": d.get("lon"),
                    "label": d.get("label", "unknown"),
                    "confidence": d.get("confidence", 0.0)
                }
                for d in detections if d.get("lat") and d.get("lon")
            ]
        except Exception as e:
            self.get_logger().warning(f"Detection parse error: {e}")

    # ---------------------------------------------------------------------
    def generate_map(self):
        """Generate and save an updated interactive map."""
        with self.map_lock:
            m = folium.Map(location=self.map_center, zoom_start=self.zoom_start)
            marker_cluster = MarkerCluster().add_to(m)

            # Drone marker
            if self.drone_position["lat"] and self.drone_position["lon"]:
                popup = f"Drone<br>Alt: {self.drone_position['alt']:.1f} m"
                folium.Marker(
                    [self.drone_position["lat"], self.drone_position["lon"]],
                    popup=popup,
                    icon=folium.Icon(color="blue", icon="plane", prefix="fa")
                ).add_to(marker_cluster)

            # Mission path
            if self.mission_points:
                folium.PolyLine(
                    self.mission_points,
                    color="orange",
                    weight=3,
                    opacity=0.8,
                    tooltip="Mission Path"
                ).add_to(m)

                # Add waypoint markers
                for i, (lat, lon) in enumerate(self.mission_points, start=1):
                    folium.Marker(
                        [lat, lon],
                        popup=f"Waypoint {i}",
                        icon=folium.Icon(color="green", icon="flag")
                    ).add_to(marker_cluster)

            # AI detections
            for det in self.detected_objects:
                lat, lon = det["lat"], det["lon"]
                label = det["label"]
                conf = det["confidence"]
                folium.CircleMarker(
                    location=[lat, lon],
                    radius=6,
                    color="red",
                    fill=True,
                    fill_color="red",
                    popup=f"{label} ({conf*100:.1f}%)"
                ).add_to(m)

            # Save map
            os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
            m.save(self.output_path)
            self.get_logger().info(f"🗺️ Map updated → {self.output_path}")

    # ---------------------------------------------------------------------
    def map_loop(self):
        """Continuously update the map file."""
        rate = 1.0 / max(0.01, self.update_rate)
        while self.running and rclpy.ok():
            try:
                self.generate_map()
            except Exception as e:
                self.get_logger().error(f"Map generation failed: {e}")
            time.sleep(rate)

    # ---------------------------------------------------------------------
    def destroy_node(self):
        """Graceful shutdown."""
        self.running = False
        if self.map_thread.is_alive():
            self.map_thread.join(timeout=1.0)
        super().destroy_node()


# ---------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
