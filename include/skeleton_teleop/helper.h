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

//TFs and joint names for Kinect
#define TRACKING_KINECT 1
enum {SPINEBASE, SPINEMID, NECK, HEAD, SHOULDERLEFT, ELBOWLEFT, WRISTLEFT, HANDLEFT, SHOULDERRIGHT, ELBOWRIGHT, WRISTRIGHT, HANDRIGHT, HIPLEFT, KNEELEFT, ANKLELEFT, FOOTLEFT, HIPRIGHT, KNEERIGHT, ANKLERIGHT, FOOTRIGHT, SPINESHOULDER, HANDTIPLEFT , THUMBLEFT, HANDTIPRIGHT, THUMBRIGHT, CAMERA};
vector<string> kinect_joint_names({"SPINEBASE", "SPINEMID", "NECK", "HEAD", "SHOULDERLEFT", "ELBOWLEFT", "WRISTLEFT", "HANDLEFT", "SHOULDERRIGHT", "ELBOWRIGHT", "WRISTRIGHT", "HANDRIGHT", "HIPLEFT", "KNEELEFT", "ANKLELEFT", "FOOTLEFT", "HIPRIGHT", "KNEERIGHT", "ANKLERIGHT", "FOOTRIGHT", "SPINESHOULDER", "HANDTIPLEFT", "THUMBLEFT", "HANDTIPRIGHT", "THUMBRIGHT", "camera"});

//TFs and joint names for Butepage
#define TRACKING_BUTEPAGE 2
enum {Root, Hips, LeftThigh, LeftShin, LeftFoot, LeftToe,LeftToeTip, RightThigh, RightShin, RightFoot, RightToe,RightToeTip, Spine1, Spine2, Spine3, Spine4,LeftShoulder, LeftArm, LeftForeArm, LeftHand, Neck,Head, RightShoulder, RightArm, RightForeArm, RightHand};
vector<string> butepage_joint_names({"Root", "Hips", "LeftThigh", "LeftShin", "LeftFoot", "LeftToe", "LeftToeTip", "RightThigh", "RightShin", "RightFoot", "RightToe","RightToeTip", "Spine1", "Spine2", "Spine3", "Spine4","LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand", "Neck","Head", "RightShoulder", "RightArm", "RightForeArm", "RightHand"});


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