# anthro_arm_mapping
This project aims to map human arm motion onto both spherical-rotational-spherical (SRS) and non-SRS robot arms (here the non-SRS Franka Emika Panda used). The method has been used in controlling the arms of the humanoid robot GARMI to achieve anthropomorphic motion.

The code in this repo is used for single arm control in ROS2. The needed human motion information is: Pose of shoulder, pose of wrist and postition of elbow. In this project, human motions are collected by Vicon visual tracking system while the coordinate systems are built with Vicon markers.
## Prerequisites
- [ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) installation.
- [Franka ROS2](https://support.franka.de/docs/franka_ros2.html) installation.
## Getting Started
1. Create a workspace and a src folder in the workspace (e.g. `mkdir aam_ws && cd aam_ws && mkdir src`). Clone the repo under src then go to workspace.
2. Build the package `colcon build` and source it `source install/setup.bash`.



