"""
classification.py
------------------------------------------------------------
MAYURI AI Object Classification Engine
Author: Shree Praveen(@p_rav_ee_n1082)
Date: 2025-10-22

Description:
    Handles AI-based classification of detected objects
    from video frames, LiDAR, or fused sensor data.
    Supports ONNX, TensorFlow Lite, and PyTorch models
    with automatic hardware acceleration (GPU / Edge TPU).
------------------------------------------------------------
"""

import os
import cv2
import json
import time
import torch
import numpy as np
import logging
from pathlib import Path
from threading import Lock

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    tflite = None

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object
    String = None


# ------------------------------------------------------------
# 🧠 Classifier Engine
# ------------------------------------------------------------
class ObjectClassifier(Node if rclpy else object):
    def __init__(self, model_path="ai_core/models/mobilenet_surveillance.tflite", labels_path="config/labels.json"):
        if rclpy:
            super().__init__("object_classifier")

        self.model_path = model_path
        self.labels = self._load_labels(labels_path)
        self.interpreter = None
        self.lock = Lock()

        # Logging setup
        log_dir = Path("logs/ai")
        log_dir.mkdir(parents=True, exist_ok=True)
        logging.basicConfig(
            filename=log_dir / "classification.log",
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s]: %(message)s",
        )

        # ROS2 publisher setup
        if rclpy:
            self.class_pub = self.create_publisher(String, "/mayuri/ai/classification", 10)

        logging.info(f"✅ ObjectClassifier initialized with model {model_path}")

        # Load AI model
        self._load_model(model_path)

    # --------------------------------------------------------
    # 🏷️ Load Class Labels
    # --------------------------------------------------------
    def _load_labels(self, labels_path):
        if not os.path.exists(labels_path):
            logging.warning(f"Label file not found: {labels_path}")
            return {}
        with open(labels_path, "r") as f:
            return json.load(f)

    # --------------------------------------------------------
    # ⚙️ Load Model (TFLite / PyTorch / ONNX)
    # --------------------------------------------------------
    def _load_model(self, model_path):
        ext = os.path.splitext(model_path)[1].lower()

        if "tflite" in ext and tflite:
            self.interpreter = tflite.Interpreter(model_path=model_path)
            self.interpreter.allocate_tensors()
            logging.info("Loaded TensorFlow Lite model successfully.")

        elif "pt" in ext and torch.cuda.is_available():
            self.model = torch.load(model_path, map_location="cuda")
            self.model.eval()
            logging.info("Loaded PyTorch model on GPU.")

        elif "onnx" in ext:
            import onnxruntime as ort
            self.session = ort.InferenceSession(model_path)
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            logging.info("Loaded ONNX model successfully.")

        else:
            raise RuntimeError(f"Unsupported or missing model type: {model_path}")

    # --------------------------------------------------------
    # 📷 Preprocess Input Frame
    # --------------------------------------------------------
    def _preprocess(self, frame, size=(224, 224)):
        img = cv2.resize(frame, size)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.expand_dims(img, axis=0)
        return img

    # --------------------------------------------------------
    # 🔍 Run Inference
    # --------------------------------------------------------
    def classify(self, frame):
        with self.lock:
            try:
                input_data = self._preprocess(frame)

                if hasattr(self, "interpreter") and self.interpreter:
                    input_details = self.interpreter.get_input_details()
                    output_details = self.interpreter.get_output_details()
                    self.interpreter.set_tensor(input_details[0]["index"], input_data)
                    self.interpreter.invoke()
                    output_data = self.interpreter.get_tensor(output_details[0]["index"])
                    class_id = int(np.argmax(output_data))
                    confidence = float(np.max(output_data))

                elif hasattr(self, "session"):
                    result = self.session.run([self.output_name], {self.input_name: input_data})[0]
                    class_id = int(np.argmax(result))
                    confidence = float(np.max(result))

                elif hasattr(self, "model"):
                    tensor = torch.from_numpy(np.transpose(input_data, (0, 3, 1, 2))).cuda()
                    output = self.model(tensor)
                    _, pred = torch.max(output.data, 1)
                    class_id = int(pred[0])
                    confidence = float(torch.nn.functional.softmax(output, dim=1)[0][class_id])

                else:
                    raise RuntimeError("No valid model loaded")

                label = self.labels.get(str(class_id), f"class_{class_id}")
                result = {"label": label, "confidence": round(confidence, 3)}
                self._publish_result(result)

                logging.info(f"Detected: {label} ({confidence:.2f})")
                return result

            except Exception as e:
                logging.error(f"Classification error: {e}")
                return None

    # --------------------------------------------------------
    # 📡 Publish Classification Result (ROS2)
    # --------------------------------------------------------
    def _publish_result(self, result):
        if rclpy and String:
            msg = String()
            msg.data = json.dumps(result)
            self.class_pub.publish(msg)

    # --------------------------------------------------------
    # 🧩 Continuous Camera Stream
    # --------------------------------------------------------
    def run_camera(self, camera_index=0):
        cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            print("❌ Could not access camera.")
            return

        print("🎥 Starting classification stream... Press 'q' to quit.")
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            result = self.classify(frame)
            if result:
                cv2.putText(frame, f"{result['label']} ({result['confidence']*100:.1f}%)",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("MAYURI - Object Classification", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    # --------------------------------------------------------
    # 🛑 Stop Node
    # --------------------------------------------------------
    def stop(self):
        print("🟥 Stopping Object Classifier...")
        if rclpy:
            self.destroy_node()


# ------------------------------------------------------------
# 🧭 Entry Point
# ------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="MAYURI Object Classification Engine")
    parser.add_argument("--model", type=str, default="ai_core/models/mobilenet_surveillance.tflite", help="Path to model")
    parser.add_argument("--labels", type=str, default="config/labels.json", help="Path to labels JSON")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default 0)")
    args = parser.parse_args()

    classifier = ObjectClassifier(model_path=args.model, labels_path=args.labels)
    classifier.run_camera(args.camera)

