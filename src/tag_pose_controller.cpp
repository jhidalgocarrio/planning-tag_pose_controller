#include <tag_pose_controller/tag_pose_controller.h>


TagPoseController::TagPoseController()
{}
TagPoseController::~TagPoseController()
{}

void TagPoseController::setLastDetectedPose(const apriltags2_ros::AprilTagDetection& april_tag)

{
  last_detected_tag = april_tag;
  pose = april_tag.pose.pose.pose;
}

void TagPoseController::calculateDistance()
{
  distance = sqrt(pow(pose.position.x,2) + pow(pose.position.y,2)
                         + pow(pose.position.z,2));
}

void TagPoseController::calculateRPY()
{
  tf::Quaternion quat(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
  tf::Matrix3x3 rotation_matrix(quat);
  rotation_matrix.getRPY(pitch, yaw, roll);
}

const double TagPoseController::getDistance()
{
  return distance;
}

const geometry_msgs::Vector3 TagPoseController::getRPY()
{
 geometry_msgs::Vector3 rpy;
 rpy.x = roll;
 rpy.y = pitch;
 rpy.z = yaw;
 return rpy;
}
