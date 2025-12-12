# Odometry Aided Monocular ORB-SLAM3

This repository contains a **ROS 2 workspace** integrating **ORB-SLAM3** with the **TurtleBot 4**. The goal of this project is to enhance Monocular SLAM by fusing wheel odometry data, improving tracking robustness and scale estimation in navigation tasks.

## 📂 Project Structure

The workspace is organized into the following core packages:

* **`src/ORB_SLAM3`**: The core ORB-SLAM3 library (modified for compatibility).
* **`src/ORB_SLAM3_ROS2`**: The ROS 2 wrapper nodes and interfaces for communicating with the robot.
* **`src/turtlebot4_simulator`**: Simulation environments (Gazebo/Ignition) for testing the robot in a virtual setting.

## ⚙️ Prerequisites

Ensure you have the following installed on your system:

* **OS:** Ubuntu 22.04 (Jammy) or 20.04 (Focal)
* **ROS 2 Distro:** Humble Hawksbill (or Foxy Fitzroy)
* **Dependencies:**
    * OpenCV 4.4+
    * Pangolin
    * Eigen3
    * Boost

## 🛠️ Installation

### 1. Clone the repository
```bash
git clone git@github.com:atharvanayak25/Odometry_Aided_Monocular_ORB_SLAM3.git
cd orb_slam3_odom_ws
```

## 2. Build Third-Party Libraries (ORB-SLAM3)

Before building the ROS workspace, you must build the core ORB-SLAM3 library dependencies.
```bash
cd src/ORB_SLAM3
# Give execution permission to the build script
chmod +x build.sh
# Build the library
./build.sh
```

## 3. Build the ROS 2 Workspace

Return to the root of the workspace and build using colcon.
```bash
cd ../..
colcon build --symlink-install
source install/setup.bash
```

##🚀 Usage

##Step 1: Launch the Simulation

Start the TurtleBot 4 simulation in Gazebo/Ignition.
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
```
##Step 2: Run ORB-SLAM3

Launch the SLAM node. Ensure your vocabulary file path and camera calibration file are correctly set in the configuration.
```bash
ros2 launch orb_slam3_ros2 orb_slam3_mono_inertial.launch.py
```

##🔧 Configuration

Camera Calibration: Check src/ORB_SLAM3_ROS2/params/ to ensure the camera intrinsics match the TurtleBot 4 simulation camera.

Odometry Topic: The wrapper is configured to subscribe to /odom (or the specific topic provided by the TurtleBot 4 simulator).

##🤝 Acknowledgements

ORB-SLAM3: UZ-SLAMLab/ORB_SLAM3

TurtleBot 4: Clearpath Robotics
