# Skeleton Teleop for Elenoide
This node implements a skeleton teleoperation node for operating the left and right arms of Elenoide using the output of the `skeleton_camera_aura` node. It is essentially a wrapper to listen to the transforms produced from nuitrack and convert them into `AxleCommand` messages to control Elenoide.

## Prerequisites
This codebase was developed and tested with ROS melodic. The package `skeleton_camera_aura` needs to be present.

## Running the `skeleton_teleop_node`
After building this package in your workspace using `catkin_make` and running the camera node in `skeleton_camera_aura`, run the below command
```
rosrun skeleton_teleop skeleton_teleop_node
```
It listens to the `/tf` topic and publishes the commands on the topic `/motion/axlecommand`