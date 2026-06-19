#!/usr/bin/env python3

import threading
import time

import rclpy
import cv2

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from flask import Flask, Response


# ============================================================
# Flask browser streaming server
# ============================================================

app = Flask(__name__)

latest_jpeg_frame = None
latest_frame_lock = threading.Lock()


@app.route("/")
def index():
    return """
    <html>
        <head>
            <title>Librarian Robot Vision Stream</title>
        </head>
        <body style="background-color:#111; color:white; text-align:center; font-family:Arial;">
            <h1>Librarian Robot Vision Stream</h1>
            <p>YOLO annotated camera stream from Raspberry Pi</p>
            <img src="/stream" style="width:90%; max-width:900px; border:3px solid white;">
        </body>
    </html>
    """


@app.route("/stream")
def stream():
    return Response(
        generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


def generate_mjpeg():
    global latest_jpeg_frame

    while True:
        with latest_frame_lock:
            frame = latest_jpeg_frame

        if frame is not None:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                frame +
                b"\r\n"
            )

        time.sleep(0.03)


def run_flask_server():
    app.run(host="0.0.0.0", port=8080, debug=False, threaded=True)


# ============================================================
# ROS2 Vision Node
# ============================================================

class VisionNode(Node):
    """
    Real Raspberry Pi camera + YOLO vision node.

    Publishes:
    - /vision/output/compressed
    - /vision/book_detection
    - /vision/class_names

    Also streams browser video at:
    - http://PI_IP:8080
    """

    def __init__(self):
        super().__init__("vision_node")

        self.bridge = CvBridge()

        video_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.img_pub = self.create_publisher(
            CompressedImage,
            "/vision/output/compressed",
            video_qos
        )

        self.det_pub = self.create_publisher(
            String,
            "/vision/book_detection",
            10
        )

        self.class_names_pub = self.create_publisher(
            String,
            "/vision/class_names",
            10
        )

        # Load YOLO model
        self.model = YOLO("/home/nour/best.onnx", task="detect")

        self.class_names = self.extract_class_names()
        self.publish_class_names_once()

        # Camera setup
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error("Camera could not be opened.")
        else:
            self.get_logger().info("Camera opened successfully.")

        self.latest_frame = None
        self.latest_results = None
        self.running = True
        self.last_print_time = time.time()

        # AI thread
        self.ai_thread = threading.Thread(target=self.ai_loop, daemon=True)
        self.ai_thread.start()

        # Browser stream thread
        self.flask_thread = threading.Thread(target=run_flask_server, daemon=True)
        self.flask_thread.start()

        # ROS timers
        self.timer = self.create_timer(0.033, self.publish_loop)
        self.class_timer = self.create_timer(2.0, self.publish_class_names_once)

        self.get_logger().info("Vision Node online.")
        self.get_logger().info("Browser stream available at: http://<PI_IP>:8080")
        self.get_logger().info("Example: http://192.168.8.81:8080")

    def extract_class_names(self):
        names = self.model.names

        if isinstance(names, dict):
            ordered_names = [names[i] for i in sorted(names.keys())]
        elif isinstance(names, list):
            ordered_names = names
        else:
            ordered_names = []

        self.get_logger().info("======================================")
        self.get_logger().info("YOLO MODEL CLASS NAMES DETECTED:")
        for i, name in enumerate(ordered_names):
            self.get_logger().info(f"  class {i}: {name}")
        self.get_logger().info("======================================")

        return ordered_names

    def publish_class_names_once(self):
        msg = String()
        msg.data = ",".join(self.class_names)
        self.class_names_pub.publish(msg)

    def ai_loop(self):
        while self.running:
            if self.latest_frame is not None:
                results = self.model(self.latest_frame, conf=0.4, verbose=False)
                self.latest_results = results[0]

            time.sleep(0.1)

    def get_best_detection(self):
        best_label = None
        best_conf = 0.0

        if self.latest_results is None:
            return best_label, best_conf

        if self.latest_results.boxes is None:
            return best_label, best_conf

        for b in self.latest_results.boxes:
            conf = float(b.conf[0])
            class_id = int(b.cls[0])
            label = self.model.names[class_id]

            if conf > best_conf:
                best_conf = conf
                best_label = label

        return best_label, best_conf

    def publish_loop(self):
        global latest_jpeg_frame

        ret, frame = self.cap.read()

        if not ret:
            return

        self.latest_frame = frame.copy()

        current_time = time.time()
        should_print = (current_time - self.last_print_time) > 1.0

        best_label, best_conf = self.get_best_detection()

        # Draw all detections
        if self.latest_results is not None and self.latest_results.boxes is not None:
            for b in self.latest_results.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                conf = float(b.conf[0])
                label = self.model.names[int(b.cls[0])]

                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"{label} {conf:.2f}",
                    (x1, max(0, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    2
                )

        # Publish best detection to ROS2
        if best_label is not None:
            det_msg = String()
            det_msg.data = f"{best_label},{best_conf:.2f}"
            self.det_pub.publish(det_msg)

            if should_print:
                self.get_logger().info(
                    f"YOLO BEST DETECTION: {best_label} | confidence={best_conf:.2f}"
                )
                self.last_print_time = current_time

        # Publish annotated image to ROS2
        ros_img_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpg")
        self.img_pub.publish(ros_img_msg)

        # Update browser stream frame
        success, jpeg = cv2.imencode(".jpg", frame)

        if success:
            with latest_frame_lock:
                latest_jpeg_frame = jpeg.tobytes()

    def destroy_node(self):
        self.running = False

        if hasattr(self, "ai_thread"):
            self.ai_thread.join(timeout=1.0)

        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = VisionNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()