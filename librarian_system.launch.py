#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_gui_tcp = LaunchConfiguration("use_gui_tcp")
    use_vision = LaunchConfiguration("use_vision")
    use_button = LaunchConfiguration("use_button")
    use_high_level = LaunchConfiguration("use_high_level")

    use_motor_agent = LaunchConfiguration("use_motor_agent")
    use_gripper_agent = LaunchConfiguration("use_gripper_agent")

    motor_port = LaunchConfiguration("motor_port")
    gripper_port = LaunchConfiguration("gripper_port")
    baud = LaunchConfiguration("baud")

    # ============================================================
    # Motor ESP32 micro-ROS agent
    # We keep this disabled by default because we are running it
    # manually in Terminal 1.
    # ============================================================

    motor_micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="motor_micro_ros_agent",
        output="screen",
        arguments=[
            "serial",
            "--dev",
            motor_port,
            "-b",
            baud,
            "-v6",
        ],
        condition=IfCondition(use_motor_agent),
    )

    # ============================================================
    # Gripper ESP32 micro-ROS agent
    # Disabled because gripper ESP is not connected now.
    # ============================================================

    gripper_micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="gripper_micro_ros_agent",
        output="screen",
        arguments=[
            "serial",
            "--dev",
            gripper_port,
            "-b",
            baud,
            "-v6",
        ],
        condition=IfCondition(use_gripper_agent),
    )

    # ============================================================
    # GUI TCP server node
    # ============================================================

    gui_tcp_server_node = Node(
        package="vision_pkg",
        executable="gui_tcp_server_node",
        name="gui_tcp_server_node",
        output="screen",
        condition=IfCondition(use_gui_tcp),
    )

    # ============================================================
    # HW-139 button node
    # ============================================================

    hw139_button_node = Node(
        package="vision_pkg",
        executable="hw139_button_node",
        name="hw139_button_node",
        output="screen",
        condition=IfCondition(use_button),
    )

    # ============================================================
    # Pi camera vision node
    # ============================================================

    vision_picam_node = Node(
        package="vision_pkg",
        executable="vision_picam",
        name="vision_picam",
        output="screen",
        prefix="libcamerify",
        condition=IfCondition(use_vision),
    )

    # ============================================================
    # High-level mission controller
    # ============================================================

    librarian_high_level_controller_node = Node(
        package="vision_pkg",
        executable="librarian_high_level_controller",
        name="librarian_high_level_controller",
        output="screen",
        condition=IfCondition(use_high_level),
    )

    return LaunchDescription([
        # IMPORTANT:
        # ESP32 motor topics are visible on ROS_DOMAIN_ID=0.
        SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET"),

        DeclareLaunchArgument(
            "use_gui_tcp",
            default_value="true",
            description="Start GUI TCP server node.",
        ),

        DeclareLaunchArgument(
            "use_vision",
            default_value="true",
            description="Start Pi camera vision node.",
        ),

        DeclareLaunchArgument(
            "use_button",
            default_value="true",
            description="Start HW-139 button node.",
        ),

        DeclareLaunchArgument(
            "use_high_level",
            default_value="true",
            description="Start librarian high-level controller.",
        ),

        DeclareLaunchArgument(
            "use_motor_agent",
            default_value="false",
            description="Start motor ESP32 micro-ROS agent from launch file.",
        ),

        DeclareLaunchArgument(
            "use_gripper_agent",
            default_value="false",
            description="Start gripper ESP32 micro-ROS agent from launch file.",
        ),

        DeclareLaunchArgument(
            "motor_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for motor ESP32.",
        ),

        DeclareLaunchArgument(
            "gripper_port",
            default_value="/dev/ttyUSB1",
            description="Serial port for gripper ESP32.",
        ),

        DeclareLaunchArgument(
            "baud",
            default_value="115200",
            description="micro-ROS serial baud rate.",
        ),

        motor_micro_ros_agent,
        gripper_micro_ros_agent,

        gui_tcp_server_node,
        hw139_button_node,
        vision_picam_node,
        librarian_high_level_controller_node,
    ])