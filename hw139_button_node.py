#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from gpiozero import DigitalInputDevice


GPIO_PIN = 22

# Most HW-139 / TTP223 touch modules output HIGH when touched.
# If your sensor works opposite, change this to False.
ACTIVE_HIGH = True

DEBOUNCE_SEC = 0.25


class HW139ButtonNode(Node):
    def __init__(self):
        super().__init__("hw139_button_node")

        self.book_ready_pub = self.create_publisher(
            String,
            "/book_ready",
            10
        )

        self.robot_status_pub = self.create_publisher(
            String,
            "/robot_status",
            10
        )

        # pull_up=False because the HW-139 module gives its own digital output.
        self.button = DigitalInputDevice(
            GPIO_PIN,
            pull_up=False,
            bounce_time=DEBOUNCE_SEC
        )

        self.last_press_time = 0.0
        self.last_state = False

        self.timer = self.create_timer(0.02, self.check_button)

        self.get_logger().info(f"HW-139 button node started on GPIO{GPIO_PIN}")
        self.publish_robot_status("HW-139 button ready")

    def publish_robot_status(self, text: str):
        msg = String()
        msg.data = text
        self.robot_status_pub.publish(msg)

    def publish_book_ready(self):
        msg = String()
        msg.data = "button pressed"
        self.book_ready_pub.publish(msg)

        self.publish_robot_status("Book ready button pressed")
        self.get_logger().info("BOOK_READY: button pressed")

    def is_pressed(self):
        raw_value = self.button.value

        if ACTIVE_HIGH:
            return raw_value == 1
        else:
            return raw_value == 0

    def check_button(self):
        pressed = self.is_pressed()

        # Rising edge only: publish once when button changes from not pressed to pressed.
        if pressed and not self.last_state:
            now = time.time()

            if now - self.last_press_time >= DEBOUNCE_SEC:
                self.last_press_time = now
                self.publish_book_ready()

        self.last_state = pressed


def main(args=None):
    rclpy.init(args=args)

    node = HW139ButtonNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()