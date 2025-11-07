"""
motion_tracking.py
------------------------------------------------------------
MAYURI Motion Tracking Engine
Author: Shree Praveen(@p_rav_ee_n1082)
Date: 2025-10-22

Description:
    Tracks moving objects using optical flow or centroid
    tracking techniques. Integrates with Fusion Engine
    and Navigation modules to support dynamic awareness.
------------------------------------------------------------
"""

import cv2
import numpy as np
import json
import time
import logging
from pathlib import Path
from datetime import datetime

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object
    String = None


# ------------------------------------------------------------
# 🧩 Motion Tracker Class
# ------------------------------------------------------------
class MotionTracker(Node if rclpy else object):
    def __init__(self, max_lost=10, tracker_type="KCF"):
        if rclpy:
            super().__init__("motion_tracker")

        self.tracker_type = tracker_type
        self.trackers = cv2.MultiTracker_create()
        self.objects = {}
        self.max_lost = max_lost  # max consecutive frames without detection

        # Logging setup
        log_dir = Path("logs/ai")
        log_dir.mkdir(parents=True, exist_ok=True)
        logging.basicConfig(
            filename=log_dir / "motion_tracking.log",
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s]: %(message)s",
        )

        # ROS2 publisher
        if rclpy:
            self.track_pub = self.create_publisher(String, "/mayuri/ai/motion_tracking", 10)

        logging.info(f"✅ MotionTracker initialized using {tracker_type}")

    # --------------------------------------------------------
    # 🧠 Create a Tracker
    # --------------------------------------------------------
    def _create_tracker(self):
        trackers = {
            "KCF": cv2.TrackerKCF_create,
            "CSRT": cv2.TrackerCSRT_create,
            "MOSSE": cv2.TrackerMOSSE_create
        }
        return trackers.get(self.tracker_type, cv2.TrackerKCF_create)()

    # --------------------------------------------------------
    # 🎯 Initialize Trackers from Detected Bounding Boxes
    # --------------------------------------------------------
    def initialize(self, frame, detections):
        """
        detections: list of bounding boxes [x, y, w, h]
        """
        self.trackers = cv2.MultiTracker_create()
        for det in detections:
            tracker = self._create_tracker()
            self.trackers.add(tracker, frame, tuple(det))
        logging.info(f"Initialized {len(detections)} trackers")

    # --------------------------------------------------------
    # 🧭 Update Tracker Positions
    # --------------------------------------------------------
    def update(self, frame):
        success, boxes = self.trackers.update(frame)
        tracked_objects = []

        if success:
            for i, newbox in enumerate(boxes):
                x, y, w, h = [int(v) for v in newbox]
                tracked_objects.append({
                    "id": i,
                    "bbox": [x, y, w, h],
                    "center": [x + w // 2, y + h // 2]
                })

            self._publish_data(tracked_objects)
            logging.info(f"Tracked {len(tracked_objects)} objects.")
            return tracked_objects
        else:
            logging.warning("Tracking lost!")
            return []

    # --------------------------------------------------------
    # 📡 Publish Tracking Data
    # --------------------------------------------------------
    def _publish_data(self, tracked_objects):
        if rclpy and String:
            msg = String()
            msg.data = json.dumps({
                "timestamp": datetime.utcnow().isoformat(),
                "objects": tracked_objects
            })
            self.track_pub.publish(msg)

    # --------------------------------------------------------
    # 🧩 Live Stream Tracking (Testing Mode)
    # --------------------------------------------------------
    def run_stream(self, source=0):
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            print("❌ Could not open video source.")
            return

        print("🎥 Press SPACE to select ROI, ENTER to confirm, Q to quit.")
        ret, frame = cap.read()
        if not ret:
            print("⚠️ No frames received.")
            return

        # Manually select objects to track
        boxes = []
        while True:
            box = cv2.selectROI("MAYURI - Select Objects", frame, fromCenter=False, showCrosshair=True)
            boxes.append(box)
            print(f"Added ROI: {box}")
            key = cv2.waitKey(0)
            if key == 13:  # Enter
                break

        self.initialize(frame, boxes)
        cv2.destroyAllWindows()

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            tracked = self.update(frame)
            for obj in tracked:
                (x, y, w, h) = obj["bbox"]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"ID {obj['id']}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            cv2.imshow("MAYURI - Motion Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()

    # --------------------------------------------------------
    # 🛑 Stop Tracker
    # --------------------------------------------------------
    def stop(self):
        print("🟥 Stopping Motion Tracker...")
        if rclpy:
            self.destroy_node()


# ------------------------------------------------------------
# 🧭 Entry Point
# ------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="MAYURI Motion Tracking Module")
    parser.add_argument("--source", type=str, default="0", help="Camera index or video path")
    parser.add_argument("--tracker", type=str, default="KCF", help="Tracker type: KCF, CSRT, MOSSE")
    args = parser.parse_args()

    tracker = MotionTracker(tracker_type=args.tracker)
    tracker.run_stream(int(args.source) if args.source.isdigit() else args.source)
