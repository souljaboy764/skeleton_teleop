#include <ctime>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
 #include <tf2/convert.h>
 #include <tf2/impl/convert.h>
 #include <tf2/transform_datatypes.h>
#include <sstream>
#include <map>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

enum {global, joint_head, joint_neck, joint_torso, joint_waist, joint_left_shoulder, joint_left_elbow, joint_left_hand, joint_right_shoulder, joint_right_elbow, joint_right_hand, joint_left_hip, joint_left_knee, joint_left_ankle, joint_right_hip, joint_right_knee, joint_right_ankle};
vector<string> joint_names({"map", "perception_nui_1_1_head", "perception_nui_1_1_neck", "perception_nui_1_1_torso", "perception_nui_1_1_waist", "perception_nui_1_1_left_shoulder", "perception_nui_1_1_left_elbow", "perception_nui_1_1_left_hand", "perception_nui_1_1_right_shoulder", "perception_nui_1_1_right_elbow", "perception_nui_1_1_right_hand", "perception_nui_1_1_left_hip", "perception_nui_1_1_left_knee", "perception_nui_1_1_left_ankle", "perception_nui_1_1_right_hip", "perception_nui_1_1_right_knee", "perception_nui_1_1_right_ankle"});
enum {LeftShoulderPitch=23, LeftShoulderYaw, LeftShoulderRoll, LeftElbow, RightShoulderPitch=37, RightShoulderYaw, RightShoulderRoll, RightElbow};


void euler_from_quaternion(geometry_msgs::Quaternion q, double &yaw, double &pitch, double &roll)
{
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	yaw = atan2(siny_cosp, cosy_cosp);
}

tf2::Transform fromMsg(geometry_msgs::Transform msg)
{
	return tf2::Transform(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));
}

tf2::Transform fromMsg(geometry_msgs::TransformStamped msg)
{
	return fromMsg(msg.transform);
}