#!/usr/bin/env python3

import socket
import threading
from typing import Set

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose2D


class GuiTcpServerNode(Node):
    def __init__(self):
        super().__init__("gui_tcp_server_node")

        # ================= TCP SETTINGS =================
        self.host = "0.0.0.0"
        self.port = 5000

        self.server_socket = None
        self.running = True

        # IMPORTANT:
        # Do NOT call this self.clients because rclpy Node already has a property named clients.
        self.tcp_clients: Set[socket.socket] = set()
        self.tcp_clients_lock = threading.Lock()

        # ================= ROS PUBLISHERS =================
        self.gui_command_pub = self.create_publisher(String, "/gui_command", 10)
        self.gui_selected_book_pub = self.create_publisher(String, "/gui_selected_book", 10)

        # ================= ROS SUBSCRIBERS =================
        self.robot_status_sub = self.create_subscription(
            String,
            "/robot_status",
            self.robot_status_callback,
            10,
        )

        # Stable detected book from the high-level controller.
        self.gui_detected_book_sub = self.create_subscription(
            String,
            "/gui_detected_book",
            self.gui_detected_book_callback,
            10,
        )

        # Kept for compatibility if another node still publishes /detected_book.
        self.detected_book_sub = self.create_subscription(
            String,
            "/detected_book",
            self.detected_book_callback,
            10,
        )

        # Kept for compatibility, but GUI should mainly trust /gui_detected_book.
        self.vision_detection_sub = self.create_subscription(
            String,
            "/vision/book_detection",
            self.vision_detection_callback,
            10,
        )

        self.robot_pose_sub = self.create_subscription(
            Pose2D,
            "/robot_pose",
            self.robot_pose_callback,
            10,
        )

        # Manual-mode display values from ESP serial bridge.
        self.manual_pose_sub = self.create_subscription(
            String,
            "/manual_pose",
            self.manual_pose_callback,
            10,
        )

        # ================= INTERNAL STATE =================
        self.last_detected_book = ""
        self.last_robot_status = "Idle"
        self.last_pose = Pose2D()
        self.last_manual_pose = ""

        # Start TCP server thread
        self.server_thread = threading.Thread(target=self.start_tcp_server, daemon=True)
        self.server_thread.start()

        self.get_logger().info("GUI TCP server node started.")
        self.get_logger().info(f"Listening on {self.host}:{self.port}")

    # ============================================================
    # TCP SERVER
    # ============================================================
    def start_tcp_server(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.server_socket.settimeout(1.0)

            self.get_logger().info(f"GUI TCP server listening on {self.host}:{self.port}")

            while self.running:
                try:
                    client_socket, client_address = self.server_socket.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                self.get_logger().info(f"GUI connected from {client_address}")

                with self.tcp_clients_lock:
                    self.tcp_clients.add(client_socket)

                self.send_to_client(client_socket, f"ROBOT_STATUS:{self.last_robot_status}")
                if self.last_detected_book:
                    self.send_to_client(client_socket, f"DETECTED_BOOK:{self.last_detected_book}")
                self.send_to_client(
                    client_socket,
                    f"POSE:{self.last_pose.x:.2f},{self.last_pose.y:.2f},{self.last_pose.theta:.2f}",
                )
                if self.last_manual_pose:
                    self.send_to_client(client_socket, f"MANUAL_POSE:{self.last_manual_pose}")

                client_thread = threading.Thread(
                    target=self.handle_client,
                    args=(client_socket, client_address),
                    daemon=True,
                )
                client_thread.start()

        except Exception as e:
            self.get_logger().error(f"TCP server error: {e}")

    def handle_client(self, client_socket: socket.socket, client_address):
        buffer = ""

        try:
            while self.running:
                data = client_socket.recv(1024)

                if not data:
                    break

                buffer += data.decode("utf-8", errors="ignore")

                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    command = line.strip()

                    if command:
                        self.get_logger().info(f"GUI command received: {command}")
                        self.handle_gui_command(command)

        except ConnectionResetError:
            self.get_logger().warn(f"GUI disconnected suddenly: {client_address}")
        except Exception as e:
            self.get_logger().error(f"Client handling error: {e}")
        finally:
            self.remove_client(client_socket)
            self.get_logger().info(f"GUI disconnected: {client_address}")

    def remove_client(self, client_socket: socket.socket):
        with self.tcp_clients_lock:
            if client_socket in self.tcp_clients:
                self.tcp_clients.remove(client_socket)

        try:
            client_socket.close()
        except Exception:
            pass

    def send_to_client(self, client_socket: socket.socket, message: str):
        try:
            if not message.endswith("\n"):
                message += "\n"
            client_socket.sendall(message.encode("utf-8"))
        except Exception:
            self.remove_client(client_socket)

    def broadcast_to_gui(self, message: str):
        if not message.endswith("\n"):
            message += "\n"

        dead_clients = []

        with self.tcp_clients_lock:
            for client_socket in list(self.tcp_clients):
                try:
                    client_socket.sendall(message.encode("utf-8"))
                except Exception:
                    dead_clients.append(client_socket)

        for client_socket in dead_clients:
            self.remove_client(client_socket)

    # ============================================================
    # GUI COMMAND HANDLING
    # ============================================================
    def handle_gui_command(self, command: str):
        msg = String()

        # ---------------- BOOK SELECTION ----------------
        if command.startswith("SELECT_BOOK:"):
            selected_book = command.split("SELECT_BOOK:", 1)[1].strip()

            selected_msg = String()
            selected_msg.data = selected_book
            self.gui_selected_book_pub.publish(selected_msg)

            cmd_msg = String()
            cmd_msg.data = "SELECT_BOOK"
            self.gui_command_pub.publish(cmd_msg)

            self.broadcast_to_gui("ROBOT_STATUS:Book selected")
            self.get_logger().info(f"Selected book published: {selected_book}")
            return

        # ---------------- MANUAL MODE COMMANDS ----------------
        # LEFT/RIGHT support press-and-hold:
        #   MANUAL_LEFT_START  -> high-level sends ESP "left"
        #   MANUAL_LEFT_STOP   -> high-level sends ESP "manual_stop"
        #   MANUAL_RIGHT_START -> high-level sends ESP "right"
        #   MANUAL_RIGHT_STOP  -> high-level sends ESP "manual_stop"
        # The STOP button is still available as MANUAL_STOP for stopping mid manual mode.
        valid_manual_commands = {
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

        if command in valid_manual_commands:
            msg.data = command
            self.gui_command_pub.publish(msg)

            status_map = {
                "MANUAL_MODE_START": "Manual mode started",
                "MANUAL_FORWARD": "Manual forward",
                "MANUAL_BACKWARD": "Manual backward",
                "MANUAL_LEFT": "Manual left",
                "MANUAL_RIGHT": "Manual right",
                "MANUAL_LEFT_START": "Manual left rotating",
                "MANUAL_LEFT_STOP": "Manual left stopped",
                "MANUAL_RIGHT_START": "Manual right rotating",
                "MANUAL_RIGHT_STOP": "Manual right stopped",
                "MANUAL_STOP": "Manual stopped",
                "MANUAL_RETURN_INITIAL_POSITION": "Returning to manual start position",
            }

            self.broadcast_to_gui(f"ROBOT_STATUS:{status_map[command]}")
            self.get_logger().info(f"Manual command published: {command}")
            return

        # ---------------- UNKNOWN COMMAND ----------------
        self.get_logger().warn(f"Unknown GUI command: {command}")
        self.broadcast_to_gui("ROBOT_STATUS:Error")

    # ============================================================
    # ROS CALLBACKS TO GUI
    # ============================================================
    def robot_status_callback(self, msg: String):
        self.last_robot_status = msg.data
        self.broadcast_to_gui(f"ROBOT_STATUS:{msg.data}")

    def gui_detected_book_callback(self, msg: String):
        book_name = self.normalize_book_name(msg.data)
        self.last_detected_book = book_name
        self.broadcast_to_gui(f"DETECTED_BOOK:{book_name}")

    def detected_book_callback(self, msg: String):
        book_name = self.normalize_book_name(msg.data)
        self.last_detected_book = book_name
        self.broadcast_to_gui(f"DETECTED_BOOK:{book_name}")

    def vision_detection_callback(self, msg: String):
        """
        Vision may publish:
        night-of-terror,0.95
        how-to-write-a-love-story,0.95
        lost-time-is-never-found-again,0.95

        GUI expects:
        Night of Terror
        How to Write a Love Story
        Lost Time is Never Found Again
        """

        raw = msg.data.strip()

        if not raw:
            return

        book_key = raw.split(",", 1)[0].strip()
        book_name = self.normalize_book_name(book_key)

        if book_name:
            self.last_detected_book = book_name
            self.broadcast_to_gui(f"DETECTED_BOOK:{book_name}")

    def robot_pose_callback(self, msg: Pose2D):
        self.last_pose = msg
        self.broadcast_to_gui(f"POSE:{msg.x:.2f},{msg.y:.2f},{msg.theta:.2f}")

    def manual_pose_callback(self, msg: String):
        self.last_manual_pose = msg.data.strip()
        if self.last_manual_pose:
            self.broadcast_to_gui(f"MANUAL_POSE:{self.last_manual_pose}")

    # ============================================================
    # HELPERS
    # ============================================================
    def normalize_book_name(self, book: str) -> str:
        book = book.strip()

        mapping = {
            "night-of-terror": "Night of Terror",
            "night_of_terror": "Night of Terror",
            "Night of Terror": "Night of Terror",

            "how-to-write-a-love-story": "How to Write a Love Story",
            "how_to_write_a_love_story": "How to Write a Love Story",
            "How to Write a Love Story": "How to Write a Love Story",

            "lost-time-is-never-found-again": "Lost Time is Never Found Again",
            "lost_time_is_never_found_again": "Lost Time is Never Found Again",
            "Lost Time is Never Found Again": "Lost Time is Never Found Again",
        }

        return mapping.get(book, book)

    def destroy_node(self):
        self.running = False

        with self.tcp_clients_lock:
            for client_socket in list(self.tcp_clients):
                try:
                    client_socket.close()
                except Exception:
                    pass
            self.tcp_clients.clear()

        if self.server_socket is not None:
            try:
                self.server_socket.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = GuiTcpServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
