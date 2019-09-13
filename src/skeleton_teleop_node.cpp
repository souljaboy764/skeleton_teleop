#include <ros/ros.h>

#include <aura_msgs/AxleCommand.h>
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
#include <string>
#include <algorithm>

#include "skeleton_teleop/helper.h"

#include <signal.h>

using namespace std;

#define clamp(n, lo, hi) n = max(lo, min(n, hi));

ros::Publisher axlCmdPub;
aura_msgs::AxleCommand cmd;

void mySigintHandler(int sig)
{
	ROS_INFO("SHUTDOWN CALLED!!!!");
	for(int i=0;i<50;i++)
		cmd.axles[i] = -1;
	
	axlCmdPub.publish(cmd);
	ros::shutdown();
}



int main(int argc, char *argv[])
{
	ros::init(argc, argv, "skeleton_teleop",  ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	
	cmd.header.seq = 0;
	cmd.delay = 100;
	cmd.priority = 800;
	for(int i=0;i<50;i++)
		cmd.axles[i] = -1;

	axlCmdPub = nh.advertise<aura_msgs::AxleCommand>("/motion/axlecommand", 1000);
	signal(SIGINT, mySigintHandler);

	int tracking_type = TRACKING_NUITRACK;

	if(argc==2 and string(argv[1])=="kinect")
		tracking_type = TRACKING_KINECT;

	string camera_frame = (tracking_type == TRACKING_NUITRACK) ? nui_joint_names[camera] : kinect_joint_names[CAMERA];

	string waist_frame = (tracking_type == TRACKING_NUITRACK) ? nui_joint_names[joint_waist] : kinect_joint_names[SPINEBASE];

	string right_shoulder_frame = (tracking_type == TRACKING_NUITRACK) ? nui_joint_names[joint_right_shoulder] : kinect_joint_names[SHOULDERRIGHT];
	string right_elbow_frame = (tracking_type == TRACKING_NUITRACK) ? nui_joint_names[joint_right_elbow] : kinect_joint_names[ELBOWRIGHT];
	string right_hand_frame = (tracking_type == TRACKING_NUITRACK) ? nui_joint_names[joint_right_hand] : kinect_joint_names[WRISTRIGHT];

	string left_shoulder_frame = (tracking_type == TRACKING_NUITRACK) ? nui_joint_names[joint_left_shoulder] : kinect_joint_names[SHOULDERLEFT];
	string left_elbow_frame = (tracking_type == TRACKING_NUITRACK) ? nui_joint_names[joint_left_elbow] : kinect_joint_names[ELBOWLEFT];
	string left_hand_frame = (tracking_type == TRACKING_NUITRACK) ? nui_joint_names[joint_left_hand] : kinect_joint_names[WRISTLEFT];


	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	ros::Rate r(10);

	while (nh.ok())
	{
		geometry_msgs::TransformStamped transformStamped;
		try
		{
			ros::Time t = ros::Time(0);

			tf2::Transform leftShoulder = fromMsg(tfBuffer.lookupTransform(camera_frame, left_shoulder_frame, t, ros::Duration(0.1)).transform);
			tf2::Transform leftElbow = fromMsg(tfBuffer.lookupTransform(camera_frame, left_elbow_frame, t, ros::Duration(0.1)).transform);
			tf2::Transform leftHand = fromMsg(tfBuffer.lookupTransform(camera_frame, left_hand_frame, t, ros::Duration(0.1)).transform);

			tf2::Transform rightShoulder = fromMsg(tfBuffer.lookupTransform(camera_frame, right_shoulder_frame, t, ros::Duration(0.1)).transform);
			tf2::Transform rightElbow = fromMsg(tfBuffer.lookupTransform(camera_frame, right_elbow_frame, t, ros::Duration(0.1)).transform);
			tf2::Transform rightHand = fromMsg(tfBuffer.lookupTransform(camera_frame, right_hand_frame, t, ros::Duration(0.1)).transform);
			
			tf2::Transform waist = fromMsg(tfBuffer.lookupTransform(camera_frame, waist_frame, t, ros::Duration(0.1)).transform);

			tf2::Vector3 zAxisHelper = waist.getOrigin() - rightShoulder.getOrigin();
			tf2::Vector3 xAxis = leftShoulder.getOrigin() - rightShoulder.getOrigin();//right to left
			tf2::Vector3 zAxis = tf2::tf2Cross(zAxisHelper, xAxis);//out of the human(like an arrow in the back)
			tf2::Vector3 yAxis = tf2::tf2Cross(zAxis, xAxis);//like spine, but straight
			//Coordinate System in the room
			tf2::Vector3 gravity = tf2::Vector3(0, 1, 0);
			tf2::Vector3 groundX = tf2::Vector3(-1, 0, 0);
			tf2::Vector3 groundZ = tf2::Vector3(0, 0, -1);

			xAxis = xAxis.normalize();
			yAxis = yAxis.normalize();
			zAxis = zAxis.normalize();

			tf2::Matrix3x3 transformationMatrix = tf2::Matrix3x3(xAxis.x(), xAxis.y(), xAxis.z(),
																 yAxis.x(), yAxis.y(), yAxis.z(),
																 zAxis.x(), zAxis.y(), zAxis.z());

			tf2::Transform transformationMatrixTF2(transformationMatrix);																 
			
			gravity = transformationMatrix*gravity;
			groundX = transformationMatrix*groundX;
			groundZ = transformationMatrix*groundZ;

			leftShoulder = transformationMatrixTF2*leftShoulder;
			leftElbow = transformationMatrixTF2*leftElbow;
			leftHand = transformationMatrixTF2*leftHand;

			rightShoulder = transformationMatrixTF2*rightShoulder;
			rightElbow = transformationMatrixTF2*rightElbow;
			rightHand = transformationMatrixTF2*rightHand;
			
			waist = transformationMatrixTF2*waist;
			

			xAxis = tf2::Vector3(1, 0, 0);
			yAxis = tf2::Vector3(0, 1, 0);
			zAxis = tf2::Vector3(0, 0, 1);

			//create help planes
			tf2::Vector3 frontView = tf2::tf2Cross(xAxis, yAxis);//Plain for front view: normal is zAxis
			tf2::Vector3 sideView = tf2::tf2Cross(yAxis, zAxis);//Plain for side view: normal is xAxis
			tf2::Vector3 topView = tf2::tf2Cross(zAxis, xAxis);//Plain for top view: normal is yAxis

			tf2::Vector3 ground = tf2::tf2Cross(groundZ, groundX);
			
			////////////////////////////////////////////////////////////////////////////////////////
			//										Left Arm									  //
			////////////////////////////////////////////////////////////////////////////////////////
			
			//Recreating arm with upper and under arm
			tf2::Vector3 leftUpperArm = leftElbow.getOrigin() - leftShoulder.getOrigin();
			tf2::Vector3 leftUnderArm = leftHand.getOrigin() - leftElbow.getOrigin();

			double leftYaw = 0;
			double leftPitch = 0;
			double leftRoll = 0;
			double leftElbowAngle = tf2::tf2Angle(leftUpperArm, leftUnderArm);

			double armlengthLeft = leftUpperArm.length();
			leftYaw = asin(leftUpperArm.x()/armlengthLeft);//Comes from robot structure
			leftPitch = atan(leftUpperArm.z()/ -leftUpperArm.y());//Comes from robot structure
		
			
			//Because of the form of the sinus it has to be checked if the angle is bigger than 90°
			if (tf2::tf2Dot(topView, leftUpperArm) >=0)
				leftYaw = M_PI - leftYaw;

			//Recreating under Arm Position with known Angles(without roll)
			tf2::Matrix3x3 leftRotationAroundZ;
			leftRotationAroundZ.setEulerYPR(leftYaw, 0, 0);
			tf2::Matrix3x3 leftRotationAroundX;
			leftRotationAroundX.setEulerYPR(0, 0, leftPitch);
			tf2::Matrix3x3 leftElbowRotation;
			leftElbowRotation.setEulerYPR(0, 0, -leftElbowAngle);

			double underArmlengthLeft = leftUnderArm.length();
			tf2::Vector3 leftUnderArmInZeroPos(0, -underArmlengthLeft, 0);
			tf2::Vector3 leftUnderArmWithoutRoll = leftRotationAroundX*(leftRotationAroundZ*(leftElbowRotation*leftUnderArmInZeroPos));

			//calculating the angle betwenn actual under arm position and the one calculated without roll
			leftRoll = tf2::tf2Angle(leftUnderArmWithoutRoll, leftUnderArm);
			
			
			//This is a check which sign the angle has as the calculation only produces positive angles
			tf2::Matrix3x3 leftRotationAroundArm;
			leftRotationAroundArm.setEulerYPR(0, -leftRoll, 0);
			tf2::Vector3 leftShouldBeWristPos = leftRotationAroundX*(leftRotationAroundZ*(leftRotationAroundArm*(leftElbowRotation*leftUnderArmInZeroPos)));
			double l1 = tf2::tf2Distance(leftUnderArm, leftShouldBeWristPos);
			// double l1 = sqrt((leftUnderArm.x() - leftShouldBeWristPos.x())*(leftUnderArm.x() - leftShouldBeWristPos.x()) + (leftUnderArm.y() - leftShouldBeWristPos.y())*(leftUnderArm.y() - leftShouldBeWristPos.y()) + (leftUnderArm.z() - leftShouldBeWristPos.z())*(leftUnderArm.z() - leftShouldBeWristPos.z()));
			double l1saver = l1;
			leftRotationAroundArm.setEulerYPR(0, leftRoll, 0);
			leftShouldBeWristPos = leftRotationAroundX*(leftRotationAroundZ*(leftRotationAroundArm*(leftElbowRotation*leftUnderArmInZeroPos)));
			l1 = tf2::tf2Distance(leftUnderArm, leftShouldBeWristPos);
			// l1 = sqrt((leftUnderArm.x() - leftShouldBeWristPos.x())*(leftUnderArm.x() - leftShouldBeWristPos.x()) + (leftUnderArm.y() - leftShouldBeWristPos.y())*(leftUnderArm.y() - leftShouldBeWristPos.y()) + (leftUnderArm.z() - leftShouldBeWristPos.z())*(leftUnderArm.z() - leftShouldBeWristPos.z()));
			if (l1 < l1saver)
				leftRoll = -leftRoll;
			
			//As there are some singularities or inaccessible areas in the kinematic structure, 
			//this smoothes these areas out or removes them 
			leftYaw *= 180/M_PI;
			leftPitch *= 180/M_PI;
			leftRoll *= 180/M_PI;
			leftElbowAngle *= 180/M_PI;

			if (tf2::tf2Dot(topView, leftUpperArm) >=0)
			{
				leftYaw = 180 - leftYaw;	
				if ((leftPitch > 15 && leftPitch < 90)|| (leftPitch < -15 && leftPitch > -90) || leftPitch >165 || leftPitch < -165) 
					leftPitch = -90;
				
				else
				{
					double comparer = abs(leftPitch);
					if (comparer > 165) comparer = -comparer + 180;
					leftPitch = -comparer/15*90;
				}
			}
			
			cmd.axles[LeftShoulderYaw] = (int16_t)leftYaw*10;
			cmd.axles[LeftShoulderPitch] = (int16_t)leftPitch*10 + 150;
			cmd.axles[LeftShoulderRoll] = (int16_t)(leftRoll*10 + 900)/2;
			cmd.axles[LeftElbow] = (int16_t)leftElbowAngle*10;

			// clamp(cmd.axles[LeftShoulderYaw], (int16_t)0, (int16_t)250);
			// clamp(cmd.axles[LeftShoulderPitch], (int16_t)0, (int16_t)800);
			// clamp(cmd.axles[LeftShoulderRoll], (int16_t)0, (int16_t)900);
			// clamp(cmd.axles[LeftElbow], (int16_t)0, (int16_t)900);

			// cmd.axles[LeftShoulderRoll] = 900 - cmd.axles[LeftShoulderRoll];

			ROS_INFO("LEFT ANGLES: %03d %03d %03d %03d",cmd.axles[LeftShoulderYaw], cmd.axles[LeftShoulderPitch], cmd.axles[LeftShoulderRoll], cmd.axles[LeftElbow]);
			
			////////////////////////////////////////////////////////////////////////////////////////
			//										Right Arm									  //
			////////////////////////////////////////////////////////////////////////////////////////
			
			//Recreating arm with upper and under arm
			tf2::Vector3 rightUpperArm = rightElbow.getOrigin() - rightShoulder.getOrigin();
			tf2::Vector3 rightUnderArm = rightHand.getOrigin() - rightElbow.getOrigin();

			double rightYaw = 0;
			double rightPitch = 0;
			double rightRoll = 0;
			double rightElbowAngle = tf2::tf2Angle(rightUpperArm, rightUnderArm);

			double armlengthRight = rightUpperArm.length();
			rightYaw = asin(-rightUpperArm.x()/armlengthRight);//Comes from robot structure
			rightPitch = atan(rightUpperArm.z()/ -rightUpperArm.y());//Comes from robot structure
		
			
			//Because of the form of the sinus it has to be checked if the angle is bigger than 90°
			if (tf2::tf2Dot(topView, rightUpperArm) >=0)
				rightYaw = M_PI - rightYaw;

			//Recreating under Arm Position with known Angles(without roll)
			tf2::Matrix3x3 rightRotationAroundZ;
			rightRotationAroundZ.setEulerYPR(-rightYaw, 0, 0);
			tf2::Matrix3x3 rightRotationAroundX;
			rightRotationAroundX.setEulerYPR(0, 0, rightPitch);
			tf2::Matrix3x3 rightElbowRotation;
			rightElbowRotation.setEulerYPR(0, 0, -rightElbowAngle);

			double underArmlengthRight = rightUnderArm.length();
			tf2::Vector3 rightUnderArmInZeroPos(0, -underArmlengthRight, 0);
			tf2::Vector3 rightUnderArmWithoutRoll = rightRotationAroundX*(rightRotationAroundZ*(rightElbowRotation*rightUnderArmInZeroPos));

			//calculating the angle betwenn actual under arm position and the one calculated without roll
			rightRoll = tf2::tf2Angle(rightUnderArmWithoutRoll, rightUnderArm);
			
			
			//This is a check which sign the angle has as the calculation only produces positive angles
			tf2::Matrix3x3 rightRotationAroundArm;
			rightRotationAroundArm.setEulerYPR(0, -rightRoll, 0);
			tf2::Vector3 rightShouldBeWristPos = rightRotationAroundX*(rightRotationAroundZ*(rightRotationAroundArm*(rightElbowRotation*rightUnderArmInZeroPos)));
			double r1 = tf2::tf2Distance(rightUnderArm, rightShouldBeWristPos);
			// double l1 = sqrt((rightUnderArm.x() - rightShouldBeWristPos.x())*(rightUnderArm.x() - rightShouldBeWristPos.x()) + (rightUnderArm.y() - rightShouldBeWristPos.y())*(rightUnderArm.y() - rightShouldBeWristPos.y()) + (rightUnderArm.z() - rightShouldBeWristPos.z())*(rightUnderArm.z() - rightShouldBeWristPos.z()));
			double r1saver = r1;
			rightRotationAroundArm.setEulerYPR(0, rightRoll, 0);
			rightShouldBeWristPos = rightRotationAroundX*(rightRotationAroundZ*(rightRotationAroundArm*(rightElbowRotation*rightUnderArmInZeroPos)));
			r1 = tf2::tf2Distance(rightUnderArm, rightShouldBeWristPos);
			// l1 = sqrt((rightUnderArm.x() - rightShouldBeWristPos.x())*(rightUnderArm.x() - rightShouldBeWristPos.x()) + (rightUnderArm.y() - rightShouldBeWristPos.y())*(rightUnderArm.y() - rightShouldBeWristPos.y()) + (rightUnderArm.z() - rightShouldBeWristPos.z())*(rightUnderArm.z() - rightShouldBeWristPos.z()));
			if (r1 < r1saver)
			{
				ROS_INFO("GOING HERE");
				rightRoll = -rightRoll;
			}
			
			//As there are some singularities or inaccessible areas in the kinematic structure, 
			//this smoothes these areas out or removes them 
			rightYaw *= 180/M_PI;
			rightPitch *= 180/M_PI;
			rightRoll *= 180/M_PI;
			rightElbowAngle *= 180/M_PI;

			rightRoll += 90;

			if (tf2::tf2Dot(topView, rightUpperArm) >=0)
			{
				rightYaw = 180 - rightYaw;	
				if ((rightPitch > 15 && rightPitch < 90)|| (rightPitch < -15 && rightPitch > -90) || rightPitch >165 || rightPitch < -165) 
					rightPitch = -90;
				
				else
				{
					double comparer = abs(rightPitch);
					if (comparer > 165) comparer = -comparer + 180;
					rightPitch = -comparer/15*90;
				}
			}

			cmd.axles[RightShoulderYaw] = (int16_t)rightYaw*10;
			cmd.axles[RightShoulderPitch] = (int16_t)rightPitch*10 + 150;
			cmd.axles[RightShoulderRoll] = (int16_t)rightRoll*10;
			cmd.axles[RightElbow] = (int16_t)rightElbowAngle*10;

			clamp(cmd.axles[RightShoulderYaw], (int16_t)0, (int16_t)250);
			clamp(cmd.axles[RightShoulderPitch], (int16_t)0, (int16_t)800);
			clamp(cmd.axles[RightShoulderRoll], (int16_t)0, (int16_t)900);
			clamp(cmd.axles[RightElbow], (int16_t)0, (int16_t)900);

			cmd.axles[RightShoulderRoll] = 900 - cmd.axles[RightShoulderRoll];

			ROS_INFO("RIGHT ANGLES: %03d %03d %03d %03d",cmd.axles[RightShoulderYaw], cmd.axles[RightShoulderPitch], cmd.axles[RightShoulderRoll], cmd.axles[RightElbow]);
			
			cmd.header.stamp = ros::Time::now();
			cmd.header.seq++;
			axlCmdPub.publish(cmd);
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
