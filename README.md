# anthro_arm_mapping
This project aims to map human arm motion onto both spherical-rotational-spherical (SRS) and non-SRS robot arms. The method has been used in controlling the arms of the humanoid robot GARMI to achieve anthropomorphic motion.

The code in this repo is used for single arm control in ROS2. The needed human motion information is: Pose of shoulder, pose of wrist and postition of elbow. In this project, human motions are collected by Vicon visual tracking system while the coordinate systems are built with Vicon markers.

The robot arm used here is the non-SRS Franka Emika Panda. For other arms an adaption is needed (see **Customize arguments**). 

Video demonstration of the experiments:
https://github.com/user-attachments/assets/4c64e274-f943-435f-b431-e805f20173c4
## Prerequisites
- [ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) installation.
- (Only needed for franka arms) [Franka ROS2](https://support.franka.de/docs/franka_ros2.html) installation.
## Getting Started - Single Arm
1. Create a workspace and a src folder in the workspace (e.g. `mkdir aam_ws && cd aam_ws && mkdir src`). Clone the repo under src then go to workspace.
2. Build the package `colcon build` and source it `source install/setup.bash`.
3. Run the online anthropomorphic mapping node:
```
ros2 run anthro_arm_mapping joints_rt_pub_node
```
This node receives human motion data captured by Vicon system and publish corresponding joint position of the robot arm.

(Optional) To stream the data from Vicon, run
```
ros2 run anthro_arm_mapping vicon_udp_to_ros2
```

4. (Optional for Franka Arm) Activate interaction module:
```
ros2 run anthro_arm_mapping interaction_detect_node
```
This module will detect wether the end-effector is blocked by human/environment entity using a simple wrench threshold based collistion detection method. 
## Getting Started - Dual Arm
To run dual arm control, it is recommanded to use **two laptops/NUCs**. Make sure they are connected.

On one of them, after building (see **1** and **2** in **Getting Started - Single Arm**), run:
```
ros2 run anthro_arm_mapping dual_joints_rt_client
```
The ```dual_joints_rt_client``` will solve the IK for the right arm and publish the dual arm solutions.

On the other laptop/NUC, after building, run:
```
ros2 run anthro_arm_mapping left_joint_rt_server 
```
The ```left_joint_rt_server``` will solve the IK for the left arm.
## Customize arguments
Configurable parameters are located in `/config`. 
1. Robot model: Robot models can be editted in `/robot_models.yaml`. 
2. Arguments: General arguments are defined in `config.yaml`, which include the name of the robot model that will be used during mapping. These parameters can also be configured with parser, e.g.:
```
ros2 run anthro_arm_mapping joints_rt_pub_node --freq 25
```




