#!/usr/bin/env python  

import rospy

from geometry_msgs.msg import TransformStamped, Transform

from tf2_msgs.msg import TFMessage
from tf2_ros import *
import tf2_py
from tf2_msgs.msg import *
from tf.transformations import *

import moveit_commander
from moveit_msgs.msg import DisplayRobotState

import numpy as np
import sys

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)

kinect_joint_names = ["SPINEBASE", "SPINEMID", "NECK", "HEAD", "SHOULDERLEFT", "ELBOWLEFT", "WRISTLEFT", "HANDLEFT", "SHOULDERRIGHT", "ELBOWRIGHT", "WRISTRIGHT", "HANDRIGHT", "HIPLEFT", "KNEELEFT", "ANKLELEFT", "FOOTLEFT", "HIPRIGHT", "KNEERIGHT", "ANKLERIGHT", "FOOTRIGHT", "SPINESHOULDER", "HANDTIPLEFT", "THUMBLEFT", "HANDTIPRIGHT", "THUMBRIGHT", "camera"]
butepage_joint_names = ["Root", "Hips", "LeftThigh", "LeftShin", "LeftFoot", "LeftToe", "LeftToeTip", "RightThigh", "RightShin", "RightFoot", "RightToe","RightToeTip", "Spine1", "Spine2", "Spine3", "Spine4","LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand", "Neck","Head", "RightShoulder", "RightArm", "RightForeArm", "RightHand"]

kinect_joints = enum(*kinect_joint_names)
butepage_joints = enum(*butepage_joint_names)

def transform_matrix(transform):
	m = quaternion_matrix([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
	m[:3,3] = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
	return m

def transform_translation(transform):
	return np.array([transform.translation.x, transform.translation.y, transform.translation.z])

def angle(a,b):
	dot = np.dot(a,b)
	return np.arccos(dot/(np.linalg.norm(a)*np.linalg.norm(b)))

def projectToPlane(plane, vec):
	return (vec - plane)*np.dot(plane,vec)

rospy.init_node('pepper_moveit_pynode', anonymous=True, disable_signals=False)

if len(sys.argv)>1 and sys.argv[1]=="kinect":
	prefix = "nturgbd_skeleton_1_"
	camera_frame = kinect_joint_names[kinect_joints.CAMERA]

	waist_frame = prefix + kinect_joint_names[kinect_joints.SPINEBASE]

	right_shoulder_frame = prefix + kinect_joint_names[kinect_joints.SHOULDERRIGHT]
	right_elbow_frame = prefix + kinect_joint_names[kinect_joints.ELBOWRIGHT]
	right_hand_frame = prefix + kinect_joint_names[kinect_joints.WRISTRIGHT]

	left_shoulder_frame = prefix + kinect_joint_names[kinect_joints.SHOULDERLEFT]
	left_elbow_frame = prefix + kinect_joint_names[kinect_joints.ELBOWLEFT]
	left_hand_frame = prefix + kinect_joint_names[kinect_joints.WRISTLEFT]

elif len(sys.argv)>1 and sys.argv[1]=="butepage":
	prefix = "butepage_skeleton_1_"
	camera_frame = "base_footprint"

	waist_frame = prefix + butepage_joint_names[butepage_joints.Hips]

	right_shoulder_frame = prefix + butepage_joint_names[butepage_joints.RightArm]
	right_elbow_frame = prefix + butepage_joint_names[butepage_joints.RightForeArm]
	right_hand_frame = prefix + butepage_joint_names[butepage_joints.RightHand]

	left_shoulder_frame = prefix + butepage_joint_names[butepage_joints.LeftArm]
	left_elbow_frame = prefix + butepage_joint_names[butepage_joints.LeftForeArm]
	left_hand_frame = prefix + butepage_joint_names[butepage_joints.LeftHand]

else:
	rospy.loginfo("Usage: rosrun skeleton_teleop skeleton_teleop_pepper_moveit_node.py (kinect|butepage)")
	rospy.signal_shutdown("Usage: rosrun skeleton_teleop skeleton_teleop_pepper_moveit_node.py (kinect|butepage)")
	exit(-1)

moveit_commander.roscpp_initialize(sys.argv)
pub = rospy.Publisher("display_robot_state", DisplayRobotState, queue_size=5)

# Initialize Moveit Interfaces
PLANNING_GROUP = "right_arm"
if len(sys.argv)>2:
	PLANNING_GROUP = sys.argv[2]
move_group = moveit_commander.MoveGroupCommander(PLANNING_GROUP)
rospy.loginfo("CREATED MOVEGROUP")

tfBuffer = tf2_ros.Buffer()
tfListener = tf2_ros.TransformListener(tfBuffer)
r = rospy.Rate(10)

msg = DisplayRobotState()
msg.state.joint_state.name = move_group.get_active_joints()

joint_group_positions  = move_group.get_current_joint_values()

while not rospy.is_shutdown():

	transformStamped = TransformStamped()
	try:
		t = rospy.Time(0)
		leftShoulder = transform_translation(tfBuffer.lookup_transform(camera_frame, left_shoulder_frame, t, rospy.Duration(0.1)).transform)
		leftElbow = transform_translation(tfBuffer.lookup_transform(camera_frame, left_elbow_frame, t, rospy.Duration(0.1)).transform)
		leftHand = transform_translation(tfBuffer.lookup_transform(camera_frame, left_hand_frame, t, rospy.Duration(0.1)).transform)

		rightShoulder = transform_translation(tfBuffer.lookup_transform(camera_frame, right_shoulder_frame, t, rospy.Duration(0.1)).transform)
		rightElbow = transform_translation(tfBuffer.lookup_transform(camera_frame, right_elbow_frame, t, rospy.Duration(0.1)).transform)
		rightHand = transform_translation(tfBuffer.lookup_transform(camera_frame, right_hand_frame, t, rospy.Duration(0.1)).transform)
		
		waist = transform_translation(tfBuffer.lookup_transform(camera_frame, waist_frame, t, rospy.Duration(0.1)).transform)

		xAxisHelper = waist - rightShoulder
		yAxis = leftShoulder - rightShoulder # right to left
		xAxis = np.cross(xAxisHelper, yAxis) # out of the human(like an arrow in the back)
		zAxis = np.cross(xAxis, yAxis) # like spine, but straight
		
		xAxis /= np.linalg.norm(xAxis)
		yAxis /= np.linalg.norm(yAxis)
		zAxis /= np.linalg.norm(zAxis)

		transformationMatrixTF2 = np.array([[xAxis[0], xAxis[1], xAxis[2]],
									 [yAxis[0], yAxis[1], yAxis[2]],
									 [zAxis[0], zAxis[1], zAxis[2]]])

		leftShoulder = np.dot(transformationMatrixTF2,leftShoulder)
		leftElbow = np.dot(transformationMatrixTF2,leftElbow)
		leftHand = np.dot(transformationMatrixTF2,leftHand)

		rightShoulder = np.dot(transformationMatrixTF2,rightShoulder)
		rightElbow = np.dot(transformationMatrixTF2,rightElbow)
		rightHand = np.dot(transformationMatrixTF2,rightHand)
		
		waist = np.dot(transformationMatrixTF2,waist)
		
		xAxis = np.array([1, 0, 0])
		yAxis = np.array([0, 1, 0])
		zAxis = np.array([0, 0, 1])

		leftYaw = 0
		leftPitch = 0
		leftRoll = 0
		leftElbowAngle = 0

		rightYaw = 0
		rightPitch = 0
		rightRoll = 0
		rightElbowAngle = 0

		if PLANNING_GROUP == "both_arms" or PLANNING_GROUP =="left_arm":
		
			###########################################
			#				Left Arm				  #
			###########################################
			
			# Recreating arm with upper and under arm
			leftUpperArm = leftElbow - leftShoulder
			leftUnderArm = leftHand - leftElbow

			leftElbowAngle = angle(leftUpperArm, leftUnderArm)

			armlengthLeft = np.linalg.norm(leftUpperArm)
			leftYaw = np.arctan2(leftUpperArm[1],-leftUpperArm[2]) #Comes from robot structure
			leftYaw += 0.009
			leftPitch = np.arctan2(leftUpperArm[0], leftUpperArm[2]) #Comes from robot structure
			leftPitch += np.pi/2

			# Recreating under Arm Position with known Angles(without roll)
			leftRotationAroundY = euler_matrix(0, leftPitch, 0)[:3,:3]
			leftRotationAroundX = euler_matrix(0, 0, leftYaw)[:3,:3]
			leftElbowRotation = euler_matrix(0, 0, leftElbowAngle)[:3,:3]

			leftUnderArmInZeroPos = np.array([np.linalg.norm(leftUnderArm), 0, 0.])
			leftUnderArmWithoutRoll = np.dot(leftRotationAroundY,np.dot(leftRotationAroundX,np.dot(leftElbowRotation,leftUnderArmInZeroPos)))
			
			# Calculating the angle betwenn actual under arm position and the one calculated without roll
			leftRoll = angle(leftUnderArmWithoutRoll, leftUnderArm)
			
			
			# This is a check which sign the angle has as the calculation only produces positive angles
			leftRotationAroundArm = euler_matrix(0, 0, -leftRoll)[:3, :3]
			leftShouldBeWristPos = np.dot(leftRotationAroundY,np.dot(leftRotationAroundX,np.dot(leftRotationAroundArm,np.dot(leftElbowRotation,leftUnderArmInZeroPos))))
			l1saver = np.linalg.norm(leftUnderArm - leftShouldBeWristPos)
			
			leftRotationAroundArm = euler_matrix(0, 0, leftRoll)[:3, :3]
			leftShouldBeWristPos = np.dot(leftRotationAroundY,np.dot(leftRotationAroundX,np.dot(leftRotationAroundArm,np.dot(leftElbowRotation,leftUnderArmInZeroPos))))
			l1 = np.linalg.norm(leftUnderArm - leftShouldBeWristPos)
			
			if l1 > l1saver:
				leftRoll = -leftRoll
			
			# leftElbowAngle = np.clip(leftElbowAngle, 0.009, 1.562)
		
		if PLANNING_GROUP == "both_arms" or PLANNING_GROUP == "right_arm":
		
			###########################################
			#				Right Arm				  #
			###########################################
			
			# Recreating arm with upper and under arm
			rightUpperArm = rightElbow - rightShoulder
			rightUnderArm = rightHand - rightElbow

			RElbowRoll = angle(rightUpperArm, rightUnderArm)

			armlengthRight = np.linalg.norm(rightUpperArm)
			
			rightUpperArm[1] = np.clip(rightUpperArm[1], -armlengthRight, 0.0) # limiting based on the robot structure
			RShoulderRoll = np.pi/2 - np.arccos(rightUpperArm[1]/armlengthRight)
			RShoulderRoll = np.arctan2(np.sin(RShoulderRoll), np.cos(RShoulderRoll))

			RShoulderPitch = np.arctan2(rightUpperArm[0], rightUpperArm[2]) # Comes from robot structure
			RShoulderPitch -= np.pi/2
			RShoulderPitch = np.arctan2(np.sin(RShoulderPitch), np.cos(RShoulderPitch))
			
			# Recreating under Arm Position with known Angles(without roll)
			rightRotationAroundY = euler_matrix(0, RShoulderPitch, 0,)[:3,:3]
			rightRotationAroundZ = euler_matrix(0, 0, RShoulderRoll)[:3,:3]
			rightElbowRotation = euler_matrix(0, 0, RElbowRoll)[:3,:3]

			rightUnderArmInZeroPos = np.array([np.linalg.norm(rightUnderArm), 0, 0.])
			rightUnderArmWithoutRoll = np.dot(rightRotationAroundY,np.dot(rightRotationAroundZ,np.dot(rightElbowRotation,rightUnderArmInZeroPos)))

			# Calculating the angle betwenn actual under arm position and the one calculated without roll
			RElbowYaw = angle(rightUnderArmWithoutRoll, rightUnderArm)
			RElbowYaw = np.arctan2(np.sin(RElbowYaw), np.cos(RElbowYaw))

			RShoulderPitch = np.clip(RShoulderPitch, -2.0857, 2.0857)
			RShoulderRoll = np.clip(RShoulderRoll, -1.5621, -0.009)
			RElbowYaw = np.clip(RElbowYaw, -2.0857, 2.0857)
			RElbowRoll = np.clip(RElbowRoll, 0.009, 1.5621)

		if PLANNING_GROUP =="both_arms":
			joint_group_positions[0] = leftPitch
			joint_group_positions[1] = leftYaw
			joint_group_positions[2] = leftRoll
			joint_group_positions[3] = leftElbowAngle
			joint_group_positions[5] = RShoulderPitch
			joint_group_positions[6] = RShoulderRoll
			joint_group_positions[7] = RElbowYaw
			joint_group_positions[8] = RElbowRoll
		
		elif PLANNING_GROUP == "right_arm":
			joint_group_positions[0] = RShoulderPitch
			joint_group_positions[1] = RShoulderRoll
			joint_group_positions[2] = RElbowYaw
			joint_group_positions[3] = RElbowRoll
		
		elif PLANNING_GROUP == "left_arm":
			joint_group_positions[0] = leftPitch
			joint_group_positions[1] = leftYaw
			joint_group_positions[2] = leftRoll
			joint_group_positions[3] = leftElbowAngle
		
		msg.state.joint_state.position = joint_group_positions
		pub.publish(msg)
		r.sleep()
	
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
		rospy.logwarn("%s",str(ex))
		rospy.Rate(1.0).sleep()
		continue