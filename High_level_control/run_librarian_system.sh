#!/bin/bash

# =====================================================
# LIBRARY RETURN ROBOT - ONE TERMINAL STARTUP SCRIPT
# This keeps the ROS2 launch file requirement.
# It starts:
#   1) motor ESP32 serial bridge
#   2) gripper micro-ROS agent
#   3) ROS2 launch file for high-level system
# =====================================================

echo ""
echo "======================================"
echo "LIBRARY ROBOT SYSTEM STARTING"
echo "======================================"
echo ""

# ===============================
# USER SETTINGS
# Change these if USB ports swap
# ===============================
MOTOR_PORT="/dev/ttyUSB0"
GRIPPER_PORT="/dev/ttyUSB1"
BAUD="115200"

ROS2_WS="/home/nour/ros2_ws"
UROS_WS="/home/nour/uros_ws"

LAUNCH_PACKAGE="vision_pkg"
LAUNCH_FILE="librarian_system.launch.py"

# Set this to false if the gripper ESP32 is not connected
RUN_GRIPPER_AGENT=true

# ===============================
# CLEAN OLD PROCESSES
# ===============================
echo "[1/6] Killing old running processes..."

pkill -f esp_serial_ros_bridge.py
pkill -f micro_ros_agent
pkill -f librarian_high_level_controller
pkill -f gui_tcp_server_node
pkill -f vision_picam
pkill -f hw139_button_node

sleep 2

# ===============================
# ROS ENVIRONMENT
# ===============================
echo "[2/6] Sourcing ROS2 environment..."

source /opt/ros/jazzy/setup.bash
source "$ROS2_WS/install/setup.bash"

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# ===============================
# USB PERMISSIONS
# ===============================
echo "[3/6] Checking USB ports..."

echo "Available ports:"
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

if [ -e "$MOTOR_PORT" ]; then
    echo "Giving permission to motor port: $MOTOR_PORT"
    sudo chmod 666 "$MOTOR_PORT"
else
    echo "ERROR: Motor port not found: $MOTOR_PORT"
    echo "Check using: ls /dev/ttyUSB* /dev/ttyACM*"
    exit 1
fi

if [ "$RUN_GRIPPER_AGENT" = true ]; then
    if [ -e "$GRIPPER_PORT" ]; then
        echo "Giving permission to gripper port: $GRIPPER_PORT"
        sudo chmod 666 "$GRIPPER_PORT"
    else
        echo "WARNING: Gripper port not found: $GRIPPER_PORT"
        echo "Continuing without gripper agent."
        RUN_GRIPPER_AGENT=false
    fi
fi

# ===============================
# START MOTOR SERIAL BRIDGE
# ===============================
echo "[4/6] Starting motor ESP32 serial ROS bridge..."

source /opt/ros/jazzy/setup.bash
source "$ROS2_WS/install/setup.bash"

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

python3 "$ROS2_WS/esp_serial_ros_bridge.py" --port "$MOTOR_PORT" --baud "$BAUD" &
MOTOR_BRIDGE_PID=$!

sleep 2

# ===============================
# START GRIPPER MICRO-ROS AGENT
# ===============================
if [ "$RUN_GRIPPER_AGENT" = true ]; then
    echo "[5/6] Starting gripper micro-ROS agent..."

    source /opt/ros/jazzy/setup.bash
    source "$UROS_WS/install/local_setup.bash"

    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset ROS_LOCALHOST_ONLY
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

    ros2 run micro_ros_agent micro_ros_agent serial --dev "$GRIPPER_PORT" -b "$BAUD" -v6 &
    GRIPPER_AGENT_PID=$!

    sleep 2
else
    echo "[5/6] Skipping gripper micro-ROS agent."
    GRIPPER_AGENT_PID=""
fi

# ===============================
# START ROS2 LAUNCH FILE
# ===============================
echo "[6/6] Starting ROS2 launch file..."

source /opt/ros/jazzy/setup.bash
source "$ROS2_WS/install/setup.bash"

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE" use_motor_agent:=false use_gripper_agent:=false &
LAUNCH_PID=$!

echo ""
echo "======================================"
echo "FULL SYSTEM IS RUNNING IN ONE TERMINAL"
echo "======================================"
echo "Motor bridge PID: $MOTOR_BRIDGE_PID"
echo "Gripper agent PID: ${GRIPPER_AGENT_PID:-not running}"
echo "Launch PID: $LAUNCH_PID"
echo ""
echo "Motor port: $MOTOR_PORT"
echo "Gripper port: ${GRIPPER_PORT:-not used}"
echo ""
echo "Keep this terminal open."
echo "Press CTRL+C to stop everything."
echo "======================================"
echo ""

# ===============================
# CLEAN STOP WHEN CTRL+C
# ===============================
cleanup() {
    echo ""
    echo "Stopping full system..."

    if [ -n "$MOTOR_BRIDGE_PID" ]; then
        kill "$MOTOR_BRIDGE_PID" 2>/dev/null
    fi

    if [ -n "$GRIPPER_AGENT_PID" ]; then
        kill "$GRIPPER_AGENT_PID" 2>/dev/null
    fi

    if [ -n "$LAUNCH_PID" ]; then
        kill "$LAUNCH_PID" 2>/dev/null
    fi

    pkill -f esp_serial_ros_bridge.py
    pkill -f micro_ros_agent
    pkill -f librarian_high_level_controller
    pkill -f gui_tcp_server_node
    pkill -f vision_picam
    pkill -f hw139_button_node

    echo "System stopped."
    exit 0
}

trap cleanup SIGINT

wait
