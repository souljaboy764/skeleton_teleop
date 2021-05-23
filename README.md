# Skeleton Teleop for Pepper
This node implements a skeleton teleoperation node for operating the left and right arms of Pepper from TFs. It is essentially a wrapper to listen to the transforms and publish a simulated robot state.

## Prerequisites
This codebase was developed and tested with ROS melodic. There needs to be a node running publishing skeleton TFs either in a Nuitrack or a Kinect format.

## Running the `skeleton_teleop_node`
After building this package in your workspace using `catkin_make` and running a node that's publishing the TFs, run the below command
```
rosrun skeleton_teleop node (kinect|butepage)
```
It listens to the `/tf` topic and publishes a `DisplayRobotState` message.

Compatible with the following packages for publishing skeletons:
https://github.com/souljaboy764/nturgbd_skeleton/
https://github.com/souljaboy764/human_robot_interaction_data

### TODO:
- Add a node integrated with NAOqi
- Integrate Robot control with MoveIt/NAOqi

## Acknowledgements
This code is adapted for pepper from the code at https://github.com/robertocalandra/firstperson-teleoperation