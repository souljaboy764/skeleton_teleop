# Skeleton Teleop for Elenoide
This node implements a skeleton teleoperation node for operating the left and right arms of Elenoide from TFs. It is essentially a wrapper to listen to the transforms produced from nuitrack/kinect and convert them into `AxleCommand` messages to control Elenoide.

## Prerequisites
This codebase was developed and tested with ROS melodic. There needs to be a node running publishing skeleton TFs either in a Nuitrack or a Kinect format.

## Running the `skeleton_teleop_node`
After building this package in your workspace using `catkin_make` and running a node that's publishing the TFs, run the below command
```
rosrun skeleton_teleop node (kinect|nuitrack)
```
If no extra argument is given, it assumes nuitrack by default. It listens to the `/tf` topic and publishes the commands on the topic `/motion/axlecommand`