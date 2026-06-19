#!/usr/bin/env python3
import re
import time
import threading
import argparse

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class ESPSerialROSBridge(Node):
    """
    Raspberry Pi ROS2 bridge for a NORMAL Arduino Serial ESP32 code.

    ESP32 code stays unchanged:
      - ESP receives commands from Serial, like path1/path2/path3/r90/l90/d50/s
      - ESP prints status/yaw/manual pose using Serial.print()

    This bridge:
      - subscribes to /esp32_motor/cmd and writes the command to ESP Serial
      - reads ESP Serial prints and republishes:
          /esp32_motor/status
          /esp32_motor/raw
          /esp32_imu/yaw_deg
          /esp32_imu/yaw
          /manual_pose
    """

    def __init__(self, port: str, baud: int):
        super().__init__("esp_serial_ros_bridge")

        self.port = port
        self.baud = baud

        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(2.0)

        self.status_pub = self.create_publisher(String, "/esp32_motor/status", 10)
        self.raw_pub = self.create_publisher(String, "/esp32_motor/raw", 10)
        self.yaw_deg_pub = self.create_publisher(Float32, "/esp32_imu/yaw_deg", 10)
        self.yaw_pub = self.create_publisher(Float32, "/esp32_imu/yaw", 10)
        self.manual_pose_pub = self.create_publisher(String, "/manual_pose", 10)

        self.cmd_sub = self.create_subscription(
            String,
            "/esp32_motor/cmd",
            self.cmd_callback,
            10
        )

        self.stop_event = threading.Event()
        self.reader_thread = threading.Thread(target=self.serial_reader_loop, daemon=True)
        self.reader_thread.start()

        self.get_logger().info(f"ESP serial ROS bridge started on {self.port} at {self.baud}")
        self.get_logger().info("Send: ros2 topic pub --once /esp32_motor/cmd std_msgs/msg/String \"{data: 'path1'}\"")
        self.get_logger().info("Echo: ros2 topic echo /esp32_imu/yaw_deg")
        self.get_logger().info("Echo: ros2 topic echo /esp32_imu/yaw")
        self.get_logger().info("Echo: ros2 topic echo /manual_pose")

    def cmd_callback(self, msg: String):
        cmd = msg.data.strip()
        if not cmd:
            return

        try:
            self.ser.write((cmd + "\n").encode())
            self.ser.flush()
            self.get_logger().info(f"Sent to ESP: {cmd}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command to ESP: {e}")

    def publish_float(self, pub, value: float):
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)

    def parse_and_publish_yaw_values(self, line: str):
        """
        Robust parsing for different Serial.print styles.

        Supported examples:
          yawDeg=1.23
          yawDeg: 1.23
          Linear yawDeg: 1.23
          yaw=-4.56
          yaw: -4.56
          MOVE ... | yaw: 0.52        -> treated as yawDeg because linear movement uses yawDeg
          BACKWARD ... | yaw: 0.52    -> treated as yawDeg
          rotState: ... | yaw: -20.1  -> treated as rotation yaw
        """

        # Explicit yawDeg labels
        m = re.search(r"\byawDeg\s*[:=]\s*(-?\d+(?:\.\d+)?)", line)
        if m:
            self.publish_float(self.yaw_deg_pub, float(m.group(1)))

        # Linear yaw correction print: "Linear yawDeg: ..."
        m = re.search(r"\bLinear\s+yawDeg\s*[:=]\s*(-?\d+(?:\.\d+)?)", line)
        if m:
            self.publish_float(self.yaw_deg_pub, float(m.group(1)))

        # Generic "| yaw:" line:
        # In your original code, MOVE/BACKWARD yaw belongs to yawDeg.
        # In rotation printData(), rotState line yaw belongs to rotation yaw.
        m = re.search(r"\byaw\s*[:=]\s*(-?\d+(?:\.\d+)?)", line)
        if m:
            yaw_value = float(m.group(1))

            if "rotState" in line or "ROTATION" in line or "rotating" in line:
                self.publish_float(self.yaw_pub, yaw_value)
            elif "MOVE" in line or "BACKWARD" in line or "Linear yawDeg" in line:
                self.publish_float(self.yaw_deg_pub, yaw_value)
            else:
                # Unknown yaw line. Publish it to rotation yaw by default,
                # because the topic name /esp32_imu/yaw is usually checked for rotation.
                self.publish_float(self.yaw_pub, yaw_value)

    def parse_and_publish_manual_pose(self, line: str):
        """
        ESP display-only line format:
          MANUAL_POSE:x_cm=12.34,y_cm=-5.67,theta_deg=22.50,last_yaw_deg=-22.50

        The TCP GUI server forwards this to Qt as:
          MANUAL_POSE:x_cm=...,y_cm=...,theta_deg=...,last_yaw_deg=...
        """
        if not line.startswith("MANUAL_POSE:"):
            return

        payload = line.replace("MANUAL_POSE:", "", 1).strip()
        if not payload:
            return

        msg = String()
        msg.data = payload
        self.manual_pose_pub.publish(msg)

    def serial_reader_loop(self):
        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode(errors="replace").strip()
                if not line:
                    continue

                # Print ESP output on Pi terminal, like Serial Monitor.
                print(line, flush=True)

                # Publish whole line as raw + status.
                raw_msg = String()
                raw_msg.data = line
                self.raw_pub.publish(raw_msg)

                status_msg = String()
                status_msg.data = line
                self.status_pub.publish(status_msg)

                # Extract values if present.
                self.parse_and_publish_yaw_values(line)
                self.parse_and_publish_manual_pose(line)

            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.2)

    def destroy_node(self):
        self.stop_event.set()
        time.sleep(0.2)
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0", help="ESP port, e.g. /dev/ttyUSB0 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    rclpy.init()
    node = ESPSerialROSBridge(args.port, args.baud)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
