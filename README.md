
# Autonomous Library Book Return Robot

## Overview

This project presents an autonomous mobile robot designed to automate the process of returning books to their correct locations in a library. The system combines autonomous navigation, computer vision, embedded control, and a graphical user interface (GUI) to create a complete book return solution.

The robot operates from a designated home position where a user selects the book to be returned through a custom GUI. After placing the book inside the gripper, the user activates the system using a touch sensor mounted on the robot.

A Raspberry Pi 4 performs high-level decision making, computer vision, and ROS 2 coordination, while two ESP32 microcontrollers handle the low-level motor control. One ESP32 communicates through a custom serial ROS bridge for mobile robot motion, while a second ESP32 runs micro-ROS to control the lifting mechanism and parallel gripper.

The onboard camera verifies that the inserted book matches the user's GUI selection. If the detected book differs from the selected one, the system automatically corrects the destination and follows the appropriate return path.

After reaching the correct shelf location, the lifting mechanism lowers the gripper, releases the book into the return slot, retracts, and the robot autonomously returns to its home position ready for the next operation.

The current implementation supports three different book return locations and also provides a manual control mode for testing and maintenance.

---

## Features

* Autonomous library book return
* Computer vision-based book identification
* Automatic correction of incorrect GUI selections
* ROS 2 distributed architecture
* Raspberry Pi 4 high-level controller
* Dual ESP32 embedded controllers
* Custom serial-to-ROS bridge
* micro-ROS communication for gripper subsystem
* Camera-based book verification
* Autonomous navigation along predefined paths
* Parallel gripper with lifting mechanism
* Manual driving mode through the GUI
* Real-time robot status monitoring
* Live robot pose estimation relative to the home position
* Single-command startup script for the complete system

---

## System Workflow

1. Robot waits at the home position.
2. User selects one of the available books from the GUI.
3. User inserts the book into the gripper.
4. User touches the capacitive touch sensor.
5. Gripper closes securely around the book.
6. Camera captures and identifies the book.
7. If necessary, the selected destination is automatically corrected.
8. Robot follows the corresponding navigation path.
9. Robot reaches the target bookshelf.
10. Lift mechanism lowers the gripper.
11. Gripper releases the book.
12. Lift retracts.
13. Robot returns to the home position.

---

## Manual Mode

The system also includes a manual operating mode accessible through the GUI.

Manual controls include:

* Start Manual Mode
* Forward
* Backward
* Continuous Left Rotation
* Continuous Right Rotation
* Stop

The GUI additionally displays:

* Current robot status
* Current operation stage
* Estimated X-Y position
* Robot heading
* Mission progress

---

## System Architecture

### High-Level Controller

* Raspberry Pi 4
* ROS 2 Jazzy
* Python nodes
* GUI TCP server
* Computer vision
* Mission controller

### Mobile Base Controller

* ESP32
* Standard Arduino Serial firmware
* Differential drive control
* IMU feedback
* Serial communication through a custom ROS bridge

### Manipulator Controller

* ESP32
* micro-ROS
* NEMA lifting motor
* Servo positioning
* Parallel gripper control

---

## Hardware

* Raspberry Pi 4
* 2 × ESP32 development boards
* Raspberry Pi Camera
* Differential drive mobile platform
* NEMA stepper motor
* DC motor
* Parallel gripper
* HW-139 capacitive touch sensor
* IMU
* Three library return locations

---

## Software

* ROS 2 Jazzy
* Python
* C++ / Arduino
* micro-ROS
* OpenCV
* TCP/IP communication
* Serial communication
* Qt-based GUI

---

## Future Improvements

* Dynamic path planning
* Shelf localization using SLAM
* Additional book categories
* Barcode or RFID verification
* Multi-book handling
* Cloud database integration
* Autonomous charging dock