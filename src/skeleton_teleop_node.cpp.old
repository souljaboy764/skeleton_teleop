#include <ros/ros.h>

#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2/impl/convert.h>

#include <iostream>

#include "skeleton_teleop/helper.h"

using namespace std;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "skeleton_teleop");
	ros::NodeHandle nh;
	
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	ros::Rate r(10);
	while (nh.ok())
	{
		geometry_msgs::TransformStamped transformStamped;
		try
		{
			ros::Time t = ros::Time(0);

			/* transformStamped = tfBuffer.lookupTransform(joint_names[joint_left_shoulder], joint_names[joint_left_elbow], t, ros::Duration(0.1));
			tf2::Quaternion q_leftElbow(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);

			transformStamped = tfBuffer.lookupTransform(joint_names[joint_right_shoulder], joint_names[joint_right_elbow], t, ros::Duration(0.1));
			tf2::Quaternion q_rightElbow(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);

			transformStamped = tfBuffer.lookupTransform(joint_names[joint_head], joint_names[joint_left_shoulder], t, ros::Duration(0.1));
			tf2::Quaternion q_leftShoulder(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);

			transformStamped = tfBuffer.lookupTransform(joint_names[joint_head], joint_names[joint_right_shoulder], t, ros::Duration(0.1));
			tf2::Quaternion q_rightShoulder(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
 */
			geometry_msgs::Transform leftShoulder = tfBuffer.lookupTransform(joint_names[global], joint_names[joint_left_shoulder], t, ros::Duration(0.1)).transform;
			geometry_msgs::Transform leftElbow = tfBuffer.lookupTransform(joint_names[global], joint_names[joint_left_elbow], t, ros::Duration(0.1)).transform;
			geometry_msgs::Transform leftHand = tfBuffer.lookupTransform(joint_names[global], joint_names[joint_left_hand], t, ros::Duration(0.1)).transform;

			tf2::Vector3 leftUpperArm(leftElbow.translation.x - leftShoulder.translation.x, leftElbow.translation.y - leftShoulder.translation.y, leftElbow.translation.z - leftShoulder.translation.z);
			tf2::Vector3 leftUnderArm(leftHand.translation.x - leftElbow.translation.x, leftHand.translation.y - leftElbow.translation.y, leftHand.translation.z - leftElbow.translation.z);
			
			double leftElbowAngle = tf2::tf2Angle(leftUpperArm, leftUnderArm);

			double armlengthLeft = sqrt(tf2::tf2Dot(leftUpperArm, leftUpperArm));
			double leftYaw = asin(leftUpperArm.x()/armlengthLeft);//Comes from robot structure
			double leftPitch = atan2(leftUpperArm.z(), leftUpperArm.y());//Comes from robot structure

			// ROS_INFO("ELBOW ANGLES: %.3f", leftElbowAngle*180/M_PI);
			ROS_INFO("SHOULDER ANGLES: %.3f %.3f", leftYaw*180/M_PI, leftPitch*180/M_PI);
			r.sleep();	
		}
		catch (tf2::TransformException &ex) 
		{
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
	}

	return 0;
}