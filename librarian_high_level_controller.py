#!/usr/bin/env python3

import time
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from vision_pkg.librarian_config import (
    BOOKS,
    CONF_THRESHOLD,
    REQUIRED_STABLE_DETECTIONS,
)


# ============================================================
# Timing parameters
# ============================================================

PATH_START_DELAY_SEC = 2.0
GRIPPER_OPEN_WAIT_SEC = 3.0


# ============================================================
# GUI display names <-> YOLO/config keys
# ============================================================

GUI_TO_YOLO = {
    "Night of Terror": "night-of-terror",
    "How to Write a Love Story": "how-to-write-a-love-story",
    "Lost Time is Never Found Again": "lost-time-is-never-found-again",
}

YOLO_TO_GUI = {
    "night-of-terror": "Night of Terror",
    "how-to-write-a-love-story": "How to Write a Love Story",
    "lost-time-is-never-found-again": "Lost Time is Never Found Again",
}


# ============================================================
# Mission states
# ============================================================

class MissionState(Enum):
    WAIT_FOR_BOOK_SELECTION = 1

    SEND_GRIPPER_HOME_BEFORE_BOOK = 2
    WAIT_FOR_GRIPPER_HOME_BEFORE_BOOK = 3

    WAIT_FOR_BOOK_READY_BUTTON = 4
    CLOSE_GRIPPER = 5

    VERIFY_BOOK_WITH_CAMERA = 6
    SELECT_TARGET = 7
    WAIT_BEFORE_PATH_START = 8
    SEND_PATH_TO_MOTOR = 9

    WAIT_FOR_LIBRARY_ARRIVAL = 10
    STOP_MOTOR_AT_LIBRARY = 11
    OPEN_GRIPPER = 12
    WAIT_FOR_BOOK_DROPPED = 13
    RESUME_MOTOR_PATH = 14
    WAIT_FOR_PATH_DONE = 15
    MISSION_COMPLETE = 16

    SEND_GRIPPER_HOME_AFTER_PATH_DONE = 17
    WAIT_FOR_GRIPPER_HOME_AFTER_PATH_DONE = 18

    ERROR = 19

    # New sequence states:
    # HW-139 -> gripper-only close -> camera decision -> NEMA -500 lift -> path start
    WAIT_FOR_GRIPPER_CLOSED_BEFORE_CAMERA = 20
    LIFT_NEMA_AFTER_CAMERA_DECISION = 21
    WAIT_FOR_NEMA_LIFT_DONE = 22


class LibrarianHighLevelController(Node):
    """
    Final GUI-driven high-level controller.

    Inputs:
        /gui_selected_book        std_msgs/String
        /gui_command              std_msgs/String
        /book_ready               std_msgs/String
        /vision/book_detection    std_msgs/String
        /vision/class_names       std_msgs/String
        /esp32_motor/status       std_msgs/String
        /esp32_gripper/status     std_msgs/String

    Outputs:
        /robot_status             std_msgs/String
        /gui_detected_book        std_msgs/String
        /mission/status           std_msgs/String
        /mission/gui_message      std_msgs/String
        /mission/selected_path    std_msgs/String
        /mission/low_level_command std_msgs/String
        /esp32_motor/cmd          std_msgs/String
        /esp32_gripper/cmd        std_msgs/String
    """

    def __init__(self):
        super().__init__("librarian_high_level_controller")

        # ============================================================
        # GUI input topics
        # ============================================================

        self.gui_selected_book_sub = self.create_subscription(
            String,
            "/gui_selected_book",
            self.gui_selected_book_callback,
            10
        )

        self.gui_command_sub = self.create_subscription(
            String,
            "/gui_command",
            self.gui_command_callback,
            10
        )

        # ============================================================
        # Real HW-139 button topic
        # This is published by hw139_button_node.py.
        # This is REQUIRED before the gripper closes.
        # ============================================================

        self.book_ready_sub = self.create_subscription(
            String,
            "/book_ready",
            self.book_ready_callback,
            10
        )

        # ============================================================
        # Vision input topics
        # ============================================================

        self.vision_sub = self.create_subscription(
            String,
            "/vision/book_detection",
            self.vision_callback,
            10
        )

        self.class_names_sub = self.create_subscription(
            String,
            "/vision/class_names",
            self.class_names_callback,
            10
        )

        # ============================================================
        # ESP status input topics
        # ============================================================

        self.motor_status_sub = self.create_subscription(
            String,
            "/esp32_motor/status",
            self.motor_status_callback,
            10
        )

        self.gripper_status_sub = self.create_subscription(
            String,
            "/esp32_gripper/status",
            self.gripper_status_callback,
            10
        )

        # Optional obstacle input for state indicators.
        # If an ultrasonic/obstacle node publishes /obstacle_detected,
        # the high-level forwards this state to the lamps on the gripper ESP.
        self.obstacle_detected_sub = self.create_subscription(
            Bool,
            "/obstacle_detected",
            self.obstacle_detected_callback,
            10
        )

        # ============================================================
        # GUI/status output topics
        # ============================================================

        self.robot_status_pub = self.create_publisher(
            String,
            "/robot_status",
            10
        )

        self.gui_detected_book_pub = self.create_publisher(
            String,
            "/gui_detected_book",
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/mission/status",
            10
        )

        self.gui_msg_pub = self.create_publisher(
            String,
            "/mission/gui_message",
            10
        )

        self.selected_path_pub = self.create_publisher(
            String,
            "/mission/selected_path",
            10
        )

        self.low_level_cmd_pub = self.create_publisher(
            String,
            "/mission/low_level_command",
            10
        )

        # ============================================================
        # ESP command output topics
        # ============================================================

        self.motor_cmd_pub = self.create_publisher(
            String,
            "/esp32_motor/cmd",
            10
        )

        self.gripper_cmd_pub = self.create_publisher(
            String,
            "/esp32_gripper/cmd",
            10
        )

        # ============================================================
        # Mission state variables
        # ============================================================

        self.state = MissionState.WAIT_FOR_BOOK_SELECTION

        self.available_yolo_classes = []

        self.selected_book = None
        self.selected_book_gui = None

        self.detected_book = None
        self.detected_confidence = 0.0
        self.gui_detected_book_sent = False

        self.final_book = None
        self.current_path_command = None

        self.last_stable_label = None
        self.stable_detection_count = 0

        self.book_ready_received = False
        self.gripper_close_sent = False
        self.gripper_closed_received = False

        self.nema_lift_sent = False
        self.nema_lift_done = False

        self.path_delay_started = False
        self.path_delay_start_time = None

        self.gripper_open_wait_started = False
        self.gripper_open_wait_start_time = None

        self.library_arrived = False
        self.book_dropped = False
        self.path_done = False

        self.sent_stop_at_library = False
        self.sent_open_gripper = False
        self.sent_resume_path = False

        # ============================================================
        # State indicator variables
        # All lamps are connected to gripper ESP.
        # High-level sends LED commands on /esp32_gripper/cmd.
        # ============================================================

        self.last_indicator_cmd = None
        self.obstacle_indicator_active = False

        # ============================================================
        # Homing variables
        # ============================================================

        self.home_command_sent = False
        self.home_done_received = False

        # This is the latest known physical upper limit switch state.
        # It should only change when the gripper ESP or a simulated test publishes:
        #   "upper limit switch trig"
        #   "upper limit switch clear"
        self.upper_limit_switch_triggered = False

        # ============================================================
        # Logging throttle
        # ============================================================

        self.last_waiting_log_time = 0.0
        self.waiting_log_period = 2.0

        # ============================================================
        # Main state machine timer
        # ============================================================

        self.timer = self.create_timer(0.2, self.update_state_machine)

        self.publish_status("READY. GUI-driven high-level controller started.")
        self.publish_robot_status("Idle")
        self.publish_gui_message("System ready. Select a book from the GUI.")

    # ============================================================
    # GUI callbacks
    # ============================================================

    def gui_selected_book_callback(self, msg: String):
        gui_book_name = msg.data.strip()

        self.publish_status(f"GUI_SELECTED_BOOK_RECEIVED: {gui_book_name}")

        if gui_book_name not in GUI_TO_YOLO:
            self.publish_status(f"ERROR: Unknown GUI book name: {gui_book_name}")
            self.publish_robot_status(f"Unknown selected book: {gui_book_name}")
            self.publish_gui_message(f"Invalid selected book from GUI: {gui_book_name}")
            return

        yolo_book_key = GUI_TO_YOLO[gui_book_name]

        if yolo_book_key not in BOOKS:
            self.publish_status(
                f"ERROR: GUI book maps to '{yolo_book_key}', but it is not in BOOKS config."
            )
            self.publish_robot_status("Book configuration error")
            self.publish_gui_message(
                f"Configuration error: {yolo_book_key} not found in librarian_config.py."
            )
            return

        if self.state != MissionState.WAIT_FOR_BOOK_SELECTION:
            self.publish_robot_status(
                "Book selection ignored because mission is already running. Press STOP first if needed."
            )
            self.publish_gui_message(
                "Mission already running. New book selection ignored."
            )
            return

        self.reset_for_new_mission_keep_limit_state()

        self.selected_book = yolo_book_key
        self.selected_book_gui = gui_book_name

        self.publish_status(f"Selected book set: GUI='{gui_book_name}', YOLO='{yolo_book_key}'")
        self.publish_robot_status(f"Selected book: {gui_book_name}")
        self.publish_gui_message(
            f"Selected book: {gui_book_name}. Checking gripper home position."
        )

        self.reset_home_cycle_flags()
        self.state = MissionState.SEND_GRIPPER_HOME_BEFORE_BOOK

    def gui_command_callback(self, msg: String):
        cmd = msg.data.strip()

        self.publish_status(f"GUI_COMMAND_RECEIVED: {cmd}")

        if cmd == "SELECT_BOOK":
            # The actual selected book name arrives on /gui_selected_book.
            return

        # ============================================================
        # Manual mode commands from the updated Qt GUI
        # ============================================================
        # GUI commands:
        #   MANUAL_MODE_START
        #   MANUAL_FORWARD
        #   MANUAL_BACKWARD
        #   MANUAL_LEFT_START / MANUAL_LEFT_STOP
        #   MANUAL_RIGHT_START / MANUAL_RIGHT_STOP
        #   MANUAL_LEFT / MANUAL_RIGHT are still accepted as old start commands
        #   MANUAL_STOP
        #   MANUAL_RETURN_INITIAL_POSITION
        #
        # Low-level ESP commands:
        #   manual_start   -> clear manual frame and create a new initial pose
        #   forward        -> manual continuous forward
        #   backward       -> manual continuous backward
        #   left           -> continuous manual left rotation
        #   right          -> continuous manual right rotation
        #   manual_stop    -> stop/pause only and save net X/Y/theta
        #   manual_return  -> return to initial manual pose
        # ============================================================

        manual_cmds = {
            "MANUAL_MODE_START",
            "MANUAL_FORWARD",
            "MANUAL_BACKWARD",
            "MANUAL_LEFT",
            "MANUAL_RIGHT",
            "MANUAL_LEFT_START",
            "MANUAL_LEFT_STOP",
            "MANUAL_RIGHT_START",
            "MANUAL_RIGHT_STOP",
            "MANUAL_STOP",
            "MANUAL_RETURN_INITIAL_POSITION",
        }

        if cmd in manual_cmds:
            if self.state != MissionState.WAIT_FOR_BOOK_SELECTION:
                # During an automatic mission, manual movement is not allowed.
                # MANUAL_STOP is treated as emergency stop only during mission.
                if cmd == "MANUAL_STOP":
                    self.publish_motor_cmd("stop")
                    self.publish_gripper_cmd("GSTOP")
                    self.publish_robot_status("Stopped")
                    self.publish_gui_message(
                        "Manual STOP received during mission. Robot stopped and mission reset."
                    )
                    self.reset_for_new_mission_keep_limit_state()
                    self.state = MissionState.WAIT_FOR_BOOK_SELECTION
                else:
                    self.publish_robot_status(
                        "Manual command ignored during automatic mission. Use STOP if needed."
                    )
                    self.publish_gui_message(
                        f"Manual command '{cmd}' ignored because mission is running."
                    )
                return

            if cmd == "MANUAL_MODE_START":
                self.publish_motor_cmd("manual_start")
                self.publish_robot_status("Manual mode started")
                self.publish_gui_message(
                    "Manual mode started. Initial manual position saved by motor ESP."
                )

            elif cmd == "MANUAL_FORWARD":
                self.publish_motor_cmd("forward")
                self.publish_robot_status("Manual forward")
                self.publish_gui_message("Manual forward command sent.")

            elif cmd == "MANUAL_BACKWARD":
                self.publish_motor_cmd("backward")
                self.publish_robot_status("Manual backward")
                self.publish_gui_message("Manual backward command sent.")

            elif cmd == "MANUAL_LEFT" or cmd == "MANUAL_LEFT_START":
                # ESP32 motor code expects "left" for continuous manual rotation.
                self.publish_motor_cmd("left")
                self.publish_robot_status("Manual left rotating")
                self.publish_gui_message("Manual left rotation started. Release button to stop and save angle.")

            elif cmd == "MANUAL_LEFT_STOP":
                # Release stops the continuous rotation and saves the final manual yaw/theta on the ESP32.
                self.publish_motor_cmd("manual_stop")
                self.publish_robot_status("Manual left stopped")
                self.publish_gui_message("Manual left rotation stopped. Angle saved.")

            elif cmd == "MANUAL_RIGHT" or cmd == "MANUAL_RIGHT_START":
                # ESP32 motor code expects "right" for continuous manual rotation.
                self.publish_motor_cmd("right")
                self.publish_robot_status("Manual right rotating")
                self.publish_gui_message("Manual right rotation started. Release button to stop and save angle.")

            elif cmd == "MANUAL_RIGHT_STOP":
                # Release stops the continuous rotation and saves the final manual yaw/theta on the ESP32.
                self.publish_motor_cmd("manual_stop")
                self.publish_robot_status("Manual right stopped")
                self.publish_gui_message("Manual right rotation stopped. Angle saved.")

            elif cmd == "MANUAL_STOP":
                self.publish_motor_cmd("manual_stop")
                self.publish_robot_status("Manual stopped")
                self.publish_gui_message(
                    "Manual movement stopped. Current net X/Y/theta saved."
                )

            elif cmd == "MANUAL_RETURN_INITIAL_POSITION":
                self.publish_motor_cmd("manual_return")
                self.publish_robot_status("Returning to manual start position")
                self.publish_gui_message(
                    "Returning to the initial manual-mode position."
                )

            return

        self.publish_robot_status(f"Unknown GUI command: {cmd}")
        self.publish_gui_message(f"Unknown GUI command: {cmd}")

    # ============================================================
    # HW-139 /book_ready callback
    # ============================================================

    def book_ready_callback(self, msg: String):
        text = msg.data.strip().lower()

        self.publish_status(f"BOOK_READY_TOPIC_RECEIVED: {text}")

        if text not in ["button pressed", "book ready", "pressed", "ready"]:
            self.publish_status(f"Ignoring unknown /book_ready message: {text}")
            return

        if self.state == MissionState.WAIT_FOR_BOOK_READY_BUTTON:
            self.book_ready_received = True
            self.publish_robot_status("Book ready button pressed")
            self.publish_gui_message("Book ready button pressed. Closing gripper.")
            self.publish_status("Valid HW-139 book-ready signal accepted.")
        else:
            self.publish_status(
                f"HW-139 press ignored because current state is {self.state.name}."
            )
            self.publish_gui_message(
                "Book-ready button press ignored because robot is not waiting for a book."
            )

    # ============================================================
    # Vision callbacks
    # ============================================================

    def class_names_callback(self, msg: String):
        names = [x.strip().lower() for x in msg.data.split(",") if x.strip()]

        if names and names != self.available_yolo_classes:
            self.available_yolo_classes = names
            self.publish_status(f"YOLO_CLASSES_RECEIVED: {self.available_yolo_classes}")

            missing = [name for name in names if name not in BOOKS]

            if missing:
                self.publish_status(
                    f"WARNING: YOLO classes not found in librarian_config.py: {missing}"
                )
                self.publish_robot_status("YOLO/config mismatch")
                self.publish_gui_message(
                    "Configuration warning: some YOLO classes are missing from librarian_config.py."
                )

    def vision_callback(self, msg: String):
        raw = msg.data.strip().lower()

        try:
            label, conf_str = raw.split(",")
            confidence = float(conf_str)

        except ValueError:
            self.publish_status(f"WARNING: bad /vision/book_detection format: {raw}")
            return

        self.detected_book = label.strip()
        self.detected_confidence = confidence

    # ============================================================
    # Obstacle callback for state indicators
    # ============================================================

    def obstacle_detected_callback(self, msg: Bool):
        self.obstacle_indicator_active = bool(msg.data)

        if self.obstacle_indicator_active:
            self.publish_indicator_cmd("LED_OBSTACLE_ON")
            self.publish_status("INDICATOR: obstacle detected -> red lamp ON")
        else:
            self.publish_indicator_cmd("LED_OBSTACLE_OFF")
            self.publish_status("INDICATOR: obstacle cleared -> red lamp OFF")

    # ============================================================
    # ESP status callbacks
    # ============================================================

    def motor_status_callback(self, msg: String):
        status = msg.data.strip().lower()
        self.handle_motor_status(status)

    def gripper_status_callback(self, msg: String):
        status = msg.data.strip().lower()
        self.handle_gripper_status(status)

    # ============================================================
    # ESP status handlers
    # ============================================================

    def handle_motor_status(self, status: str):
        self.publish_status(f"MOTOR_STATUS_RECEIVED: {status}")

        # Optional support if motor ESP/bridge reports obstacle text in /esp32_motor/status.
        if "obstacle" in status:
            if any(word in status for word in ["clear", "removed", "no obstacle"]):
                self.obstacle_indicator_active = False
                self.publish_indicator_cmd("LED_OBSTACLE_OFF")
            else:
                self.obstacle_indicator_active = True
                self.publish_indicator_cmd("LED_OBSTACLE_ON")

        if status.endswith("arrive lib"):
            path_command = status.replace(" arrive lib", "").strip()

            if self.current_path_command and path_command != self.current_path_command:
                self.publish_status(
                    f"WARNING: arrival status '{status}' does not match current path '{self.current_path_command}'"
                )
                self.publish_robot_status(
                    f"Warning: received {status}, but current path is {self.current_path_command}."
                )
                return

            self.library_arrived = True
            self.publish_robot_status(f"Arrived at library on {path_command}")
            self.publish_gui_message(
                f"Robot arrived at library on {path_command}. Preparing to drop the book."
            )

        elif status.endswith("done"):
            path_command = status.replace(" done", "").strip()

            if self.current_path_command and path_command != self.current_path_command:
                self.publish_status(
                    f"WARNING: done status '{status}' does not match current path '{self.current_path_command}'"
                )
                self.publish_robot_status(
                    f"Warning: received {status}, but current path is {self.current_path_command}."
                )
                return

            self.path_done = True
            self.publish_robot_status(f"{path_command} done")
            self.publish_gui_message(f"{path_command} completed. Robot returned home.")

        elif status in ["stopped", "stop done"]:
            self.publish_robot_status("Motor stopped")
            self.publish_gui_message("Motor ESP reported stopped.")

        else:
            self.publish_status(f"Unrecognized motor status kept as log only: {status}")

    def handle_gripper_status(self, status: str):
        self.publish_status(f"GRIPPER_STATUS_RECEIVED: {status}")

        if status == "book dropped":
            self.book_dropped = True
            self.publish_robot_status("Book dropped")
            self.publish_gui_message("Book dropped successfully.")

        elif status in ["gripper done", "gripper object gripped", "gripper closed"]:
            if self.state == MissionState.WAIT_FOR_GRIPPER_CLOSED_BEFORE_CAMERA:
                self.gripper_closed_received = True
                self.publish_robot_status("Gripper closed")
                self.publish_gui_message("Gripper closed. Camera will now detect the book.")
                self.publish_status(
                    f"Gripper-only close confirmed before camera detection: {status}"
                )
            else:
                self.publish_status(
                    f"Gripper close/done status received outside close-before-camera state: {status}"
                )

        elif status == "nema done":
            if self.state == MissionState.WAIT_FOR_NEMA_LIFT_DONE:
                self.nema_lift_done = True
                self.publish_robot_status("NEMA lifted")
                self.publish_gui_message("NEMA moved -500. Robot is ready to start the path.")
                self.publish_status("NEMA -500 lift confirmed after camera decision.")
            else:
                self.publish_status("NEMA done received outside NEMA lift state.")

        elif status == "home done":
            self.home_done_received = True
            self.publish_robot_status("Gripper home done")
            self.publish_gui_message("Gripper home done.")

        elif status == "upper limit switch trig":
            self.upper_limit_switch_triggered = True
            self.publish_robot_status("Gripper at home position")
            self.publish_gui_message(
                "Upper limit switch triggered. Gripper is at home position."
            )

        elif status == "upper limit switch clear":
            self.upper_limit_switch_triggered = False
            self.publish_robot_status("Gripper not at home position")
            self.publish_gui_message(
                "Upper limit switch cleared. Gripper is not at home position."
            )

        elif status in ["button pressed", "book ready", "pressed", "ready"]:
            # This is optional support in case later the gripper ESP publishes
            # the physical book-ready signal instead of the Pi GPIO node.
            if self.state == MissionState.WAIT_FOR_BOOK_READY_BUTTON:
                self.book_ready_received = True
                self.publish_robot_status("Book ready button pressed")
                self.publish_gui_message(
                    "Book ready signal received from gripper ESP. Closing gripper only before camera detection."
                )
                self.publish_status("Valid gripper ESP book-ready signal accepted.")
            else:
                self.publish_status(
                    f"Gripper book-ready status ignored because current state is {self.state.name}."
                )

        else:
            self.publish_status(f"Unrecognized gripper status kept as log only: {status}")

    # ============================================================
    # Publisher helpers
    # ============================================================

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_robot_status(self, text: str):
        msg = String()
        msg.data = text
        self.robot_status_pub.publish(msg)
        self.get_logger().info(f"ROBOT_STATUS: {text}")

        self.update_indicator_from_robot_status(text)

    def publish_gui_message(self, text: str):
        msg = String()
        msg.data = text
        self.gui_msg_pub.publish(msg)
        self.get_logger().info(f"GUI_MESSAGE: {text}")

    def publish_gui_detected_book(self, book_key: str):
        gui_name = YOLO_TO_GUI.get(book_key, self.get_display_name(book_key))

        msg = String()
        msg.data = gui_name
        self.gui_detected_book_pub.publish(msg)

        self.publish_status(f"GUI_DETECTED_BOOK_SENT: {gui_name}")

    def publish_selected_path(self, book_name: str):
        cfg = BOOKS[book_name]

        text = (
            f"final_book={book_name}; "
            f"esp_path_command={cfg['esp_path_command']}; "
            f"path_pauses_at_library=True; "
            f"requires_resume_path=True"
        )

        msg = String()
        msg.data = text
        self.selected_path_pub.publish(msg)
        self.get_logger().info(f"SELECTED_PATH: {text}")

    def publish_motor_cmd(self, command: str):
        msg = String()
        msg.data = command
        self.motor_cmd_pub.publish(msg)

        self.publish_low_level_command(f"MOTOR:{command}")
        self.get_logger().info(f"MOTOR_CMD: {command}")

        self.update_indicator_from_motor_command(command)

    def publish_gripper_cmd(self, command: str):
        msg = String()
        msg.data = command
        self.gripper_cmd_pub.publish(msg)

        self.publish_low_level_command(f"GRIPPER:{command}")
        self.get_logger().info(f"GRIPPER_CMD: {command}")

    def publish_indicator_cmd(self, command: str):
        # Do not let idle/moving override obstacle red until obstacle is cleared.
        if self.obstacle_indicator_active and command not in ["LED_OBSTACLE_OFF", "LED_OBSTACLE_ON"]:
            return

        if self.last_indicator_cmd == command:
            return

        self.last_indicator_cmd = command
        self.publish_gripper_cmd(command)

    def update_indicator_from_motor_command(self, command: str):
        cmd = command.strip().lower()

        moving_cmds = {
            "path1", "path2", "path3",
            "resume path",
            "forward", "backward",
            "manual_left_start", "manual_right_start",
            "left", "right",
            "manual_return",
        }

        idle_cmds = {
            "stop", "s", "manual_stop",
            "manual_left_stop", "manual_right_stop",
            "reset",
        }

        if cmd in moving_cmds:
            self.publish_indicator_cmd("LED_ROBOT_MOVING")
        elif cmd in idle_cmds:
            self.publish_indicator_cmd("LED_IDLE")

    def update_indicator_from_robot_status(self, text: str):
        status = text.strip().lower()

        if "obstacle" in status:
            if any(word in status for word in ["clear", "removed", "no obstacle"]):
                self.obstacle_indicator_active = False
                self.publish_indicator_cmd("LED_OBSTACLE_OFF")
            else:
                self.obstacle_indicator_active = True
                self.publish_indicator_cmd("LED_OBSTACLE_ON")
            return

        moving_words = [
            "moving",
            "sending path",
            "starting path",
            "returning home",
            "returning to manual",
            "manual forward",
            "manual backward",
            "manual left rotating",
            "manual right rotating",
        ]

        idle_words = [
            "idle",
            "ready for next book",
            "mission complete",
            "manual stopped",
            "stopped",
            "waiting for book",
            "book selected",
        ]

        if any(word in status for word in moving_words):
            self.publish_indicator_cmd("LED_ROBOT_MOVING")
        elif any(word in status for word in idle_words):
            self.publish_indicator_cmd("LED_IDLE")

    def publish_low_level_command(self, command: str):
        msg = String()
        msg.data = command
        self.low_level_cmd_pub.publish(msg)
        self.get_logger().info(f"LOW_LEVEL_COMMAND: {command}")

    def publish_waiting_status_throttled(self, text: str):
        now = time.time()

        if now - self.last_waiting_log_time >= self.waiting_log_period:
            self.last_waiting_log_time = now
            self.publish_status(text)

    # ============================================================
    # Reset helpers
    # ============================================================

    def reset_home_cycle_flags(self):
        self.home_command_sent = False
        self.home_done_received = False

        # Do NOT reset upper_limit_switch_triggered here.
        # It represents the latest physical switch state.

    def reset_path_cycle_flags(self):
        self.library_arrived = False
        self.book_dropped = False
        self.path_done = False

        self.sent_stop_at_library = False
        self.sent_open_gripper = False
        self.sent_resume_path = False

        self.gripper_open_wait_started = False
        self.gripper_open_wait_start_time = None

    def reset_detection_flags(self):
        self.detected_book = None
        self.detected_confidence = 0.0
        self.gui_detected_book_sent = False

        self.final_book = None
        self.last_stable_label = None
        self.stable_detection_count = 0

    def reset_book_ready_flags(self):
        self.book_ready_received = False
        self.gripper_close_sent = False
        self.gripper_closed_received = False

        self.nema_lift_sent = False
        self.nema_lift_done = False

    def reset_for_new_mission_keep_limit_state(self):
        self.selected_book = None
        self.selected_book_gui = None
        self.current_path_command = None

        self.reset_detection_flags()
        self.reset_path_cycle_flags()
        self.reset_book_ready_flags()
        self.reset_home_cycle_flags()

        self.path_delay_started = False
        self.path_delay_start_time = None

        self.last_waiting_log_time = 0.0

    # ============================================================
    # Helper logic
    # ============================================================

    def get_display_name(self, book_key: str):
        if book_key in YOLO_TO_GUI:
            return YOLO_TO_GUI[book_key]

        if book_key in BOOKS:
            return BOOKS[book_key].get("display_name", book_key)

        return book_key

    def get_path_command(self, book_key: str):
        return BOOKS[book_key]["esp_path_command"]

    def gripper_home_ready_after_home_command(self):
        return self.home_done_received and self.upper_limit_switch_triggered

    def is_detection_stable(self):
        if self.detected_book is None:
            return False

        if self.detected_confidence < CONF_THRESHOLD:
            self.stable_detection_count = 0
            return False

        if self.detected_book == self.last_stable_label:
            self.stable_detection_count += 1
        else:
            self.last_stable_label = self.detected_book
            self.stable_detection_count = 1

        return self.stable_detection_count >= REQUIRED_STABLE_DETECTIONS

    # ============================================================
    # Main state machine
    # ============================================================

    def update_state_machine(self):
        if self.state == MissionState.WAIT_FOR_BOOK_SELECTION:
            return

        # ========================================================
        # Home check before starting a mission
        # ========================================================

        elif self.state == MissionState.SEND_GRIPPER_HOME_BEFORE_BOOK:
            self.publish_robot_status("Checking gripper home")

            if self.upper_limit_switch_triggered:
                self.publish_status(
                    "Upper limit switch already triggered. Skipping gripper home command."
                )
                self.publish_robot_status("Gripper already home")
                self.publish_gui_message(
                    "Gripper already at home position. Place the book in the gripper, then touch the HW-139 sensor."
                )

                self.home_done_received = True
                self.state = MissionState.WAIT_FOR_BOOK_READY_BUTTON
                return

            if not self.home_command_sent:
                self.publish_gripper_cmd("home")
                self.home_command_sent = True

                self.publish_robot_status("Homing gripper")
                self.publish_gui_message(
                    "Gripper is not home. Home command sent. Waiting for home done and upper limit switch."
                )
                self.publish_status(
                    "Waiting for gripper statuses: 'home done' and 'upper limit switch trig'."
                )

            self.state = MissionState.WAIT_FOR_GRIPPER_HOME_BEFORE_BOOK

        elif self.state == MissionState.WAIT_FOR_GRIPPER_HOME_BEFORE_BOOK:
            if self.gripper_home_ready_after_home_command():
                self.publish_robot_status("Waiting for book")
                self.publish_gui_message(
                    "Gripper home confirmed. Place the book in the gripper, then touch the HW-139 sensor."
                )
                self.state = MissionState.WAIT_FOR_BOOK_READY_BUTTON
            else:
                self.publish_waiting_status_throttled(
                    "Waiting for gripper home ready: need 'home done' and 'upper limit switch trig'."
                )
                return

        # ========================================================
        # Correct final behavior:
        # Wait forever for real HW-139 /book_ready signal.
        # No automatic timeout. No automatic gripper close.
        # ========================================================

        elif self.state == MissionState.WAIT_FOR_BOOK_READY_BUTTON:
            if self.book_ready_received:
                self.publish_status(
                    "Book-ready signal confirmed. Closing gripper only before camera detection."
                )
                self.publish_robot_status("Closing gripper")
                self.publish_gui_message(
                    "HW-139 pressed. Closing gripper first, then camera will detect the book."
                )

                self.gripper_close_sent = False
                self.gripper_closed_received = False

                self.state = MissionState.CLOSE_GRIPPER
                return

            self.publish_robot_status("Waiting for book")
            self.publish_waiting_status_throttled(
                "Waiting for HW-139 book-ready signal on /book_ready."
            )
            return

        elif self.state == MissionState.CLOSE_GRIPPER:
            if not self.gripper_close_sent:
                self.publish_robot_status("Closing gripper")

                # IMPORTANT:
                # Use G-2 instead of CLOSE here.
                # CLOSE in the gripper ESP also moves NEMA -500, but the new sequence
                # requires NEMA -500 only AFTER camera detection and path decision.
                self.publish_gripper_cmd("G-2")

                self.gripper_close_sent = True

                self.publish_status(
                    "Sent G-2 gripper-only close. Waiting for 'gripper done' or 'gripper object gripped'."
                )
                self.publish_gui_message(
                    "Gripper is closing. Camera detection will start after the gripper finishes."
                )

                self.state = MissionState.WAIT_FOR_GRIPPER_CLOSED_BEFORE_CAMERA

        elif self.state == MissionState.WAIT_FOR_GRIPPER_CLOSED_BEFORE_CAMERA:
            if self.gripper_closed_received:
                self.publish_status("Gripper closed. Starting camera verification.")
                self.publish_robot_status("Detecting book")
                self.publish_gui_message("Gripper closed. Detecting book with camera.")

                self.state = MissionState.VERIFY_BOOK_WITH_CAMERA
            else:
                self.publish_waiting_status_throttled(
                    "Waiting for gripper status: 'gripper done' or 'gripper object gripped'."
                )
                return

        # ========================================================
        # Camera verification + automatic wrong-book correction
        # ========================================================

        elif self.state == MissionState.VERIFY_BOOK_WITH_CAMERA:
            if self.detected_book is None:
                self.publish_robot_status("Detecting book")
                self.publish_waiting_status_throttled("Waiting for /vision/book_detection...")
                return

            if not self.is_detection_stable():
                self.publish_robot_status("Detecting book")
                self.publish_waiting_status_throttled(
                    f"Camera sees {self.detected_book} conf={self.detected_confidence:.2f}. Waiting for stable detection..."
                )
                return

            if self.detected_book not in BOOKS:
                self.publish_status(
                    f"ERROR: YOLO detected '{self.detected_book}', but it is not in BOOKS config."
                )
                self.publish_robot_status("Unknown detected book")
                self.publish_gui_message(
                    f"Detected unknown book: {self.detected_book}. Robot stopped."
                )
                self.state = MissionState.ERROR
                return

            if not self.gui_detected_book_sent:
                self.publish_gui_detected_book(self.detected_book)
                self.gui_detected_book_sent = True

            selected_display = self.get_display_name(self.selected_book)
            detected_display = self.get_display_name(self.detected_book)

            if self.detected_book == self.selected_book:
                self.final_book = self.selected_book
                path_command = self.get_path_command(self.final_book)

                self.publish_status(
                    f"SUCCESS: selected book matches detected book: {self.final_book}"
                )
                self.publish_robot_status("Correct book detected. Returning book.")
                self.publish_gui_message(
                    f"Correct book detected: {detected_display}. Preparing {path_command}."
                )

            else:
                self.final_book = self.detected_book
                path_command = self.get_path_command(self.final_book)

                self.publish_status(
                    f"MISMATCH: selected={self.selected_book}, detected={self.detected_book}."
                )
                self.publish_status(
                    f"Autocorrecting to detected book: {self.final_book}"
                )
                self.publish_robot_status(
                    "Wrong book detected, returning detected book to correct shelf"
                )
                self.publish_gui_message(
                    f"Selected book '{selected_display}' is missing. "
                    f"Camera detected '{detected_display}' instead. "
                    f"Autocorrecting to {detected_display} and selecting {path_command}."
                )

            self.state = MissionState.SELECT_TARGET

        # ========================================================
        # Select final path and start motion
        # ========================================================

        elif self.state == MissionState.SELECT_TARGET:
            self.publish_selected_path(self.final_book)
            self.current_path_command = self.get_path_command(self.final_book)

            self.path_delay_started = False
            self.path_delay_start_time = None

            self.nema_lift_sent = False
            self.nema_lift_done = False

            self.publish_robot_status("Path selected. Lifting NEMA")
            self.publish_gui_message(
                f"Path selected: {self.current_path_command}. Moving NEMA -500 before starting."
            )

            self.state = MissionState.LIFT_NEMA_AFTER_CAMERA_DECISION

        elif self.state == MissionState.LIFT_NEMA_AFTER_CAMERA_DECISION:
            if not self.nema_lift_sent:
                # Plain number command is handled by the gripper ESP as NEMA steps.
                # Negative value = NEMA down/up lifting direction according to your mechanism.
                self.publish_gripper_cmd("-500")
                self.nema_lift_sent = True

                self.publish_status(
                    "Sent NEMA -500 after camera decision. Waiting for 'nema done'."
                )
                self.publish_robot_status("Lifting NEMA")
                self.publish_gui_message(
                    "Moving NEMA -500 before starting the selected path."
                )

                self.state = MissionState.WAIT_FOR_NEMA_LIFT_DONE

        elif self.state == MissionState.WAIT_FOR_NEMA_LIFT_DONE:
            if self.nema_lift_done:
                self.publish_status(
                    f"NEMA lift done. Starting {self.current_path_command} after delay."
                )
                self.publish_robot_status("NEMA ready. Starting path soon")
                self.publish_gui_message(
                    f"NEMA lift complete. Robot will start {self.current_path_command}."
                )

                self.state = MissionState.WAIT_BEFORE_PATH_START
            else:
                self.publish_waiting_status_throttled(
                    "Waiting for gripper ESP status: 'nema done'."
                )
                return

        elif self.state == MissionState.WAIT_BEFORE_PATH_START:
            if not self.path_delay_started:
                self.path_delay_started = True
                self.path_delay_start_time = time.time()

                self.publish_status(
                    f"Waiting {PATH_START_DELAY_SEC:.1f} seconds before sending {self.current_path_command}."
                )
                self.publish_robot_status(
                    f"Starting {self.current_path_command} in {PATH_START_DELAY_SEC:.0f} seconds"
                )
                self.publish_gui_message(
                    f"Book confirmed. Robot will start {self.current_path_command} in {PATH_START_DELAY_SEC:.0f} seconds."
                )
                return

            elapsed = time.time() - self.path_delay_start_time

            if elapsed >= PATH_START_DELAY_SEC:
                self.state = MissionState.SEND_PATH_TO_MOTOR

        elif self.state == MissionState.SEND_PATH_TO_MOTOR:
            self.publish_motor_cmd(self.current_path_command)
            self.publish_robot_status(f"Sending {self.current_path_command} to motor ESP")
            self.publish_gui_message(
                f"Sent {self.current_path_command} to motor ESP. Waiting for '{self.current_path_command} arrive lib'."
            )
            self.state = MissionState.WAIT_FOR_LIBRARY_ARRIVAL

        elif self.state == MissionState.WAIT_FOR_LIBRARY_ARRIVAL:
            if self.library_arrived:
                self.state = MissionState.STOP_MOTOR_AT_LIBRARY
            else:
                self.publish_waiting_status_throttled(
                    f"Waiting for motor status: '{self.current_path_command} arrive lib'."
                )

        # ========================================================
        # Drop book at library
        # ========================================================

        elif self.state == MissionState.STOP_MOTOR_AT_LIBRARY:
            if not self.sent_stop_at_library:
                self.publish_motor_cmd("stop")
                self.sent_stop_at_library = True

                self.publish_robot_status("Robot stopped at library")
                self.publish_gui_message("Robot stopped in front of library.")

                self.state = MissionState.OPEN_GRIPPER

        elif self.state == MissionState.OPEN_GRIPPER:
            if not self.sent_open_gripper:
                self.publish_gripper_cmd("G OPEN")
                self.sent_open_gripper = True

                self.gripper_open_wait_started = True
                self.gripper_open_wait_start_time = time.time()

                self.publish_robot_status("Opening gripper")
                self.publish_gui_message(
                    f"Opening gripper. Waiting {GRIPPER_OPEN_WAIT_SEC:.0f} seconds before accepting book dropped."
                )
                self.publish_status(
                    f"Sent G OPEN to gripper. Waiting {GRIPPER_OPEN_WAIT_SEC:.1f} seconds before accepting 'book dropped'."
                )

                self.state = MissionState.WAIT_FOR_BOOK_DROPPED

        elif self.state == MissionState.WAIT_FOR_BOOK_DROPPED:
            if self.gripper_open_wait_started:
                elapsed = time.time() - self.gripper_open_wait_start_time

                if elapsed < GRIPPER_OPEN_WAIT_SEC:
                    return

            if self.book_dropped:
                self.state = MissionState.RESUME_MOTOR_PATH
            else:
                self.publish_waiting_status_throttled(
                    "Waiting for gripper status: 'book dropped'."
                )
                return

        elif self.state == MissionState.RESUME_MOTOR_PATH:
            if not self.sent_resume_path:
                self.publish_motor_cmd("resume path")
                self.sent_resume_path = True

                self.publish_robot_status("Returning home")
                self.publish_gui_message(
                    f"Book dropped. Resuming {self.current_path_command} back home."
                )
                self.state = MissionState.WAIT_FOR_PATH_DONE

        elif self.state == MissionState.WAIT_FOR_PATH_DONE:
            if self.path_done:
                self.state = MissionState.MISSION_COMPLETE
            else:
                self.publish_waiting_status_throttled(
                    f"Waiting for motor status: '{self.current_path_command} done'."
                )
                return

        # ========================================================
        # Mission complete + post-mission home check
        # ========================================================

        elif self.state == MissionState.MISSION_COMPLETE:
            self.publish_low_level_command("MISSION:DONE_RETURNED_HOME")
            self.publish_robot_status("Mission complete")
            self.publish_gui_message("Mission complete. Robot returned home.")
            self.publish_status(
                "MISSION_COMPLETE. Checking gripper home before next mission."
            )

            self.reset_home_cycle_flags()
            self.state = MissionState.SEND_GRIPPER_HOME_AFTER_PATH_DONE

        elif self.state == MissionState.SEND_GRIPPER_HOME_AFTER_PATH_DONE:
            # Force a fresh HOME after every completed path.
            # Do NOT skip even if upper_limit_switch_triggered was already True,
            # because the requested final sequence requires gripper + NEMA homing every time.
            if not self.home_command_sent:
                self.home_done_received = False
                self.upper_limit_switch_triggered = False

                self.publish_gripper_cmd("home")
                self.home_command_sent = True

                self.publish_robot_status("Post-mission homing gripper and NEMA")
                self.publish_gui_message(
                    "Path done. Re-homing gripper and NEMA before the next mission."
                )
                self.publish_status(
                    "Post-mission: forced HOME sent. Waiting for 'home done' and 'upper limit switch trig'."
                )

            self.state = MissionState.WAIT_FOR_GRIPPER_HOME_AFTER_PATH_DONE

        elif self.state == MissionState.WAIT_FOR_GRIPPER_HOME_AFTER_PATH_DONE:
            if self.gripper_home_ready_after_home_command():
                self.publish_robot_status("Ready for next book")
                self.publish_gui_message(
                    "Post-mission gripper home complete. System ready for next book."
                )
                self.publish_status(
                    "READY_FOR_NEXT_BOOK. Select a book again."
                )

                self.reset_path_cycle_flags()
                self.reset_detection_flags()
                self.reset_book_ready_flags()

                self.selected_book = None
                self.selected_book_gui = None
                self.final_book = None
                self.current_path_command = None
                self.path_delay_started = False
                self.path_delay_start_time = None

                self.state = MissionState.WAIT_FOR_BOOK_SELECTION
            else:
                self.publish_waiting_status_throttled(
                    "Post-mission waiting for gripper home ready."
                )
                return

        # ========================================================
        # Error state
        # ========================================================

        elif self.state == MissionState.ERROR:
            self.publish_motor_cmd("stop")
            self.publish_gripper_cmd("GSTOP")
            self.publish_low_level_command("MISSION:STOP")

            self.publish_robot_status("Error. Robot stopped.")
            self.publish_gui_message("Error state. Robot stopped safely.")

            self.reset_for_new_mission_keep_limit_state()
            self.state = MissionState.WAIT_FOR_BOOK_SELECTION


def main(args=None):
    rclpy.init(args=args)

    node = LibrarianHighLevelController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()