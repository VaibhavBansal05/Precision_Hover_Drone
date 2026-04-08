# 🚁 Autonomous Precision Landing Drone (Ignition Gazebo + ROS 2)

![ROS 2](https://img.shields.io/badge/ROS_2-Humble_/_Iron-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Ignition Gazebo](https://img.shields.io/badge/Ignition_Gazebo-Simulation-orange?style=for-the-badge)
![OpenCV](https://img.shields.io/badge/OpenCV-Vision_Servoing-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-blue?style=for-the-badge&logo=python&logoColor=yellow)

This repository contains an autonomous ROS 2 package (`drone_hover`) designed to perform **Vision-Based Precision Landing** utilizing ArUco fiducial markers inside an Ignition Gazebo (Gazebo Sim) environment. 

---

## 🎥 Demonstration Video
**[Watch the Mission on YouTube](https://youtu.be/KsZD5R67yig)** 
> Watch the drone autonomously take off, acquire the ArUco marker, and use proportional continuous visual servoing to lock perfectly above the target!

---

## 🏗️ Project Architecture (The `drone_hover` Package)

This repository is built as a highly robust, self-contained ROS 2 package containing:

1. **`worlds/hover_world.sdf`**
   * The complete Simulation Environment defined in SDF.
   * Spawns the `landing_pad` and the dynamically included `aruco_marker` model perfectly on top of it.
   * Instantiates an X3 Quadcopter UAV equipped with a downward-facing camera (640x480 @ 30FPS) and a 1D downward `gpu_lidar`.
   * Integrates the `MulticopterVelocityControl` plugin to accept velocity commands and actuate the propellers.

2. **`drone_hover/hover_node.py` (The Flight Brain)**
   * A continuous visual-servoing P-controller system.
   * Replaces complex state machines with elegant, magnetic-like mathematical tracking.
   * **Vertical Lock:** Constantly reads the downward Lidar and adjusts thrust to maintain a perfect hardcoded `1.0m` altitude. 
   * **Horizontal Lock:** Uses the OpenCV 4.7+ API (`ArucoDetector`) to pinpoint `DICT_4X4_50` markers. It calculates the pixel drift between the camera's true optical center and the detected marker's center, generating X/Y compensatory velocities to lock the drone perfectly dead-center.

---

## 🚀 Execution & Usage

To run the full simulation and autonomous code, compile your workspace (`colcon build`) and open **three** separate terminal windows. 

### Dependencies
Ensure you have `ros-<distro>-ros-gz` (the ROS to Ignition bridge), `cv_bridge`, and `opencv-python` installed.

### Terminal 1: Launch the Gazebo World
Launch the physical environment, lighting, the landing pad, and the X3 UAV.
```bash
ign gazebo -v 4 ~/drone_ws/src/drone_hover/worlds/hover_world.sdf
```

### Terminal 2: Boot the ROS-Ignition Bridge
Bridge the Ignition transport topics to standard ROS 2 topics so the node can see the camera and control the drone.
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image \
  /lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
  /drone/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
```

### Terminal 3: Execute the Autonomous Node
Run the built-in ROS 2 flight controller node from the package. The drone will automatically arm itself, propel up to 1.0m, and lock onto the marker.
```bash
ros2 run drone_hover hover_node
```
*(Note: Be sure your workspace is sourced (`source install/setup.bash`) before running this command).*
