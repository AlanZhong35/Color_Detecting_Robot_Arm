# Color Sorting Robot Arm

## Overview
This project integrates **computer vision** with a **6-axis robotic arm** to **autonomously sort objects based on color**. The arm is equipped with a gripper and a Logitech camera to detect the color of objects and place them in the appropriate bucket.

Our goal was to develop a **minimum viable product (MVP)** in **two weeks**, focusing on distinguishing **red and blue** objects (whiteboard erasers) and placing them in **predefined bucket locations**. This project combines **computer vision, kinematics, and robotics**, leveraging **OpenCV** for color detection and **ROS2** for control and communication. The demo video can be found at [https://youtu.be/S264jDhHa3s](https://youtu.be/S264jDhHa3s)

## System Architecture

### Color Detection
We used **OpenCV** with the **HSV color space** for robust color recognition under varying lighting conditions. The robot detects red and blue objects through **color masking** and thresholds the detected pixels to determine the object's color.

### ROS2 Integration
The robot operates using **ROS2 nodes**, where:
* `color_detect.py`: Processes camera input and publishes the detected color.
* `color_move.py`: Subscribes to color data and commands the arm to pick up and place objects accordingly.

### Kinematics & Movement
The robotic arm follows predefined **XYZ coordinates** for object pickup and placement. **Smooth movement and speed adjustments** ensure stable operation, preventing issues like overshooting or excessive force application.

## Installation
1. Ensure you have **ROS2** installed.
2. Clone this repository into your **ROS2 workspace** and build it:

```
colcon build --packages-select <package_name>
source install/setup.bash
```

## Usage

### Launching the System
Use the following **launch file** to start the system:

```
ros2 launch xarmrob xarm_keyboard.launch.py
```

### Running Nodes Individually
Run the individual nodes as needed:

```
ros2 run xarmrob color_detect
ros2 run xarmrob color_move
```

### Manual Control
For debugging and manual testing, the script `manual_control.py` allows direct control of the gripper and movement:
* `'g'` – Close gripper
* `'r'` – Release gripper
* `'block'`, `'bucket1'`, `'bucket2'` – Move to predefined locations
* `(x, y, z)` – Move to custom coordinates
* `'q'` – Confirm detected color and proceed

## Challenges & Solutions
* **Gripper Issues** – Adjusted torque limits and introduced a **gap stop** to prevent over-gripping.
* **Movement Stability** – Implemented **midpoint transitions** and **step-based speed control** for smoother operation.
* **Object Misplacement** – Introduced **delays** (`time.sleep()`) to allow precise placement before release.

## Future Improvements
* **Full Autonomy** – Enable the arm to detect and grasp objects from arbitrary locations using **edge detection**.
* **Expanded Color Detection** – Add more colors beyond red and blue, leveraging our **modular OpenCV-based approach**.
* **Conveyor Belt Integration** – Automate sorting from a moving conveyor for increased efficiency.

## Project Report
For a **detailed breakdown** of our methodology, troubleshooting, and findings, see `ProjectReport.md`.
