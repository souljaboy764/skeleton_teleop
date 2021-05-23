#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include <tf2_msgs/TFMessage.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2/impl/convert.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <iostream>
#include <string>
#include <algorithm>

#include "skeleton_teleop/helper.h"

using namespace std;

#define clamp(n, lo, hi) n = max(lo, min(n, hi));




int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pepper_moveit_cppnode");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	
	string prefix, camera_frame, waist_frame, right_shoulder_frame, right_elbow_frame, right_hand_frame, left_shoulder_frame, left_elbow_frame, left_hand_frame;

	if(argc>1 and string(argv[1])=="kinect")
	{
		prefix = "nturgbd_skeleton_1_";
		camera_frame = kinect_joint_names[CAMERA];

		waist_frame = prefix + kinect_joint_names[SPINEBASE];

		right_shoulder_frame = prefix + kinect_joint_names[SHOULDERRIGHT];
		right_elbow_frame = prefix + kinect_joint_names[ELBOWRIGHT];
		right_hand_frame = prefix + kinect_joint_names[WRISTRIGHT];

		left_shoulder_frame = prefix + kinect_joint_names[SHOULDERLEFT];
		left_elbow_frame = prefix + kinect_joint_names[ELBOWLEFT];
		left_hand_frame = prefix + kinect_joint_names[WRISTLEFT];
	}
	else if(argc>1 and string(argv[1])=="butepage")
	{
		prefix = "butepage_skeleton_1_";
		camera_frame = "base_footprint";

		waist_frame = prefix + butepage_joint_names[Hips];

		right_shoulder_frame = prefix + butepage_joint_names[RightArm];
		right_elbow_frame = prefix + butepage_joint_names[RightForeArm];
		right_hand_frame = prefix + butepage_joint_names[RightHand];

		left_shoulder_frame = prefix + butepage_joint_names[LeftArm];
		left_elbow_frame = prefix + butepage_joint_names[LeftForeArm];
		left_hand_frame = prefix + butepage_joint_names[LeftHand];
	}

	
	ros::Publisher pub = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 5);

	// Initialize Moveit Interfaces
	std::string PLANNING_GROUP = "right_arm";
	if(argc>2)
		PLANNING_GROUP = string(argv[2]);
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	ROS_INFO("CREATED MOVEGROUP");
	
	ROS_INFO("GETTING JOINTS");
	const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	ros::Rate r(10);

	moveit_msgs::DisplayRobotState msg;
	msg.state.joint_state.name = move_group.getJoints();

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	

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

			tf2::Vector3 xAxisHelper = waist.getOrigin() - rightShoulder.getOrigin();
			tf2::Vector3 yAxis = leftShoulder.getOrigin() - rightShoulder.getOrigin();//right to left
			tf2::Vector3 xAxis = tf2::tf2Cross(xAxisHelper, yAxis);//out of the human(like an arrow in the back)
			tf2::Vector3 zAxis = tf2::tf2Cross(xAxis, yAxis);//like spine, but straight
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

			double leftYaw = 0;
			double leftPitch = 0;
			double leftRoll = 0;
			double leftElbowAngle = 0;

			double rightYaw = 0;
			double rightPitch = 0;
			double rightRoll = 0;
			double rightElbowAngle = 0;

			if(!PLANNING_GROUP.compare("both_arms") or !PLANNING_GROUP.compare("left_arm"))
			{
				////////////////////////////////////////////////////////////////////////////////////////
				//										Left Arm									  //
				////////////////////////////////////////////////////////////////////////////////////////
				
				//Recreating arm with upper and under arm
				tf2::Vector3 leftUpperArm = leftElbow.getOrigin() - leftShoulder.getOrigin();
				tf2::Vector3 leftUnderArm = leftHand.getOrigin() - leftElbow.getOrigin();

				leftElbowAngle = tf2::tf2Angle(leftUpperArm, leftUnderArm);

				double armlengthLeft = leftUpperArm.length();
				leftYaw = asin(leftUpperArm.y()/armlengthLeft);//Comes from robot structure
				leftYaw += 0.009;
				leftPitch = atan2(leftUpperArm.x(), leftUpperArm.z());//Comes from robot structure
				leftPitch -= M_PI/2;

				//Recreating under Arm Position with known Angles(without roll)
				tf2::Matrix3x3 leftRotationAroundY;
				leftRotationAroundY.setEulerYPR(0, leftPitch, 0);
				tf2::Matrix3x3 leftRotationAroundX;
				leftRotationAroundX.setEulerYPR(leftYaw, 0, 0);
				tf2::Matrix3x3 leftElbowRotation;
				leftElbowRotation.setEulerYPR(leftElbowAngle, 0, 0);

				double underArmlengthLeft = leftUnderArm.length();
				tf2::Vector3 leftUnderArmInZeroPos(underArmlengthLeft, 0, 0);
				tf2::Vector3 leftUnderArmWithoutRoll = leftRotationAroundY*(leftRotationAroundX*(leftElbowRotation*leftUnderArmInZeroPos));
				//calculating the angle betwenn actual under arm position and the one calculated without roll
				leftRoll = tf2::tf2Angle(leftUnderArmWithoutRoll, leftUnderArm);
				
				
				//This is a check which sign the angle has as the calculation only produces positive angles
				tf2::Matrix3x3 leftRotationAroundArm;
				leftRotationAroundArm.setEulerYPR(0, 0, -leftRoll);
				tf2::Vector3 leftShouldBeWristPos = leftRotationAroundY*(leftRotationAroundX*(leftRotationAroundArm*(leftElbowRotation*leftUnderArmInZeroPos)));
				double l1saver = tf2::tf2Distance(leftUnderArm, leftShouldBeWristPos);
				
				leftRotationAroundArm.setEulerYPR(0, 0, leftRoll);
				leftShouldBeWristPos = leftRotationAroundY*(leftRotationAroundX*(leftRotationAroundArm*(leftElbowRotation*leftUnderArmInZeroPos)));
				double l1 = tf2::tf2Distance(leftUnderArm, leftShouldBeWristPos);
				
				if (l1 > l1saver)
					leftRoll = -leftRoll;
				
				leftElbowAngle = clamp(leftElbowAngle, 0.009, 1.562);
			}

			if(!PLANNING_GROUP.compare("both_arms") or !PLANNING_GROUP.compare("right_arm"))
			{
			
				////////////////////////////////////////////////////////////////////////////////////////
				//										Right Arm									  //
				////////////////////////////////////////////////////////////////////////////////////////
				
				//Recreating arm with upper and under arm
				tf2::Vector3 rightUpperArm = rightElbow.getOrigin() - rightShoulder.getOrigin();
				tf2::Vector3 rightUnderArm = rightHand.getOrigin() - rightElbow.getOrigin();

				rightElbowAngle = tf2::tf2Angle(rightUpperArm, rightUnderArm);

				double armlengthRight = rightUpperArm.length();
				rightYaw = atan2(rightUpperArm.y(),-rightUpperArm.z());//Comes from robot structure
				// rightYaw -= 0.009;
				rightPitch = atan2(rightUpperArm.x(), rightUpperArm.z());//Comes from robot structure
				rightPitch -= M_PI/2;
				
				//Recreating under Arm Position with known Angles(without roll)
				tf2::Matrix3x3 rightRotationAroundY;
				rightRotationAroundY.setEulerYPR(0, rightPitch, 0);
				tf2::Matrix3x3 rightRotationAroundX;
				rightRotationAroundX.setEulerYPR(rightYaw, 0, 0);
				tf2::Matrix3x3 rightElbowRotation;
				rightElbowRotation.setEulerYPR(rightElbowAngle, 0, 0);

				double underArmlengthRight = rightUnderArm.length();
				tf2::Vector3 rightUnderArmInZeroPos(underArmlengthRight,0, 0);
				tf2::Vector3 rightUnderArmWithoutRoll = rightRotationAroundY*(rightRotationAroundX*(rightElbowRotation*rightUnderArmInZeroPos));
				//calculating the angle betwenn actual under arm position and the one calculated without roll
				rightRoll = tf2::tf2Angle(rightUnderArmWithoutRoll, rightUnderArm);
				
				//This is a check which sign the angle has as the calculation only produces positive angles
				tf2::Matrix3x3 rightRotationAroundArm;
				rightRotationAroundArm.setEulerYPR(0, 0, -rightRoll);
				tf2::Vector3 rightShouldBeWristPos = rightRotationAroundY*(rightRotationAroundX*(rightRotationAroundArm*(rightElbowRotation*rightUnderArmInZeroPos)));
				double r1saver = tf2::tf2Distance(rightUnderArm, rightShouldBeWristPos);
				
				rightRotationAroundArm.setEulerYPR(0, 0, rightRoll);
				rightShouldBeWristPos = rightRotationAroundY*(rightRotationAroundX*(rightRotationAroundArm*(rightElbowRotation*rightUnderArmInZeroPos)));
				double r1 = tf2::tf2Distance(rightUnderArm, rightShouldBeWristPos);
				
				if (r1 > r1saver)
					rightRoll = -rightRoll;
				
				// rightElbowAngle = clamp(rightElbowAngle, 0.009, 1.562);
				
			}

			if(!PLANNING_GROUP.compare("both_arms")) 
			{			
				joint_group_positions[0] = leftPitch;
				joint_group_positions[1] = leftYaw;
				joint_group_positions[2] = leftRoll;
				joint_group_positions[3] = leftElbowAngle;
				joint_group_positions[5] = rightPitch;
				joint_group_positions[6] = rightYaw;
				joint_group_positions[7] = rightRoll;
				joint_group_positions[8] = rightElbowAngle;
			}
			else if(!PLANNING_GROUP.compare("right_arm"))
			{
				joint_group_positions[0] = rightPitch;
				joint_group_positions[1] = rightYaw;
				joint_group_positions[2] = rightRoll;
				joint_group_positions[3] = rightElbowAngle;
			}
			else if(!PLANNING_GROUP.compare("left_arm"))
			{
				joint_group_positions[0] = leftPitch;
				joint_group_positions[1] = leftYaw;
				joint_group_positions[2] = leftRoll;
				joint_group_positions[3] = leftElbowAngle;
			}

			msg.state.joint_state.position = joint_group_positions;
			pub.publish(msg);
			r.sleep();
		}
		catch (tf2::TransformException &ex) 
		{
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
	}

	spinner.stop();

	return 0;
}
