## TODO: Write a readme (lol)

## ROS 2 packages

The project is organised into the following ROS 2 packages:

| Package | Purpose | Key launch files |
|---|---|---|
| `aruco` | ArUco marker detection, corner detection, and pose estimation. | `aruco_real.launch.py`, `aruco_sim_pohang.launch.py`, `aruco_sim_small.launch.py` |
| `barge_control` | Barge simulation control and ROS 2 interfacing, including trajectory generation and publication of barge state. | `barge_bridge.launch.py`, `barge_pohang.launch.py` |
| `custom_msgs` | Custom ROS 2 message definitions used throughout the project. | None |
| `ekf` | Target-state estimation using an EKF and target altitude filtering. | `ekf_staged.launch.py`, `ekf_staged_mocap.launch.py`, `ekf_staged_real.launch.py` |
| `ibvs` | Feedforward IBVS controller, landing state machine, and camera/PX4 control abstractions. | `main_sim.launch.py`, `main_mocap.launch.py`, `main_outdoor_rtk.launch.py`, `sim_*.launch.py` |
| `px4_msgs` | PX4 ROS 2 message definitions. | None |
| `px4_ros2_launch` | Helper launch files for PX4 and Micro XRCE-DDS communications. | `px4.launch.py`, `microxrce.launch.py`, `microxrce_real.launch.py` |
| `utils` | General utilities for simulation bridging, camera drivers, and rosbag recording. | `gz_groundtruth.launch.py`, `rpi_global_cam.launch.py`, `rosbag_sim.launch.py`, `rosbag_real.launch.py` |


## PX4 Custom Parameters
LNDMC_XY_VEL_MAX = 5m/s # Allow landing with high horizontal drift
EKF2_REQ_HDRIFT disable

## Docker Setup
Performance tested on RTX3070 + AMD Ryzen 5-3600 using Nvidia Container Toolkit. 
Enable docker xhost for GUI: xhost +local:docker
