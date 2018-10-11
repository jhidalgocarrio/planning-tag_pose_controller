#ifndef TAG_POSE_CONTROLLER_H
#define TAG_POSE_CONTROLLER_H

#include <tag_pose_controller/PID.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>

class TagPoseController {

public:

    TagPoseController();
    ~TagPoseController();
    void setLastDetectedPose(const apriltags2_ros::AprilTagDetection& april_tag);
    void calculateDistance();
    void calculateRPY();
    const double getDistance();
    const geometry_msgs::Vector3 getRPY();

private:

    PID pid;
    apriltags2_ros::AprilTagDetection  last_detected_tag;
    geometry_msgs::Pose pose;
    double distance, roll, pitch, yaw;

};


#endif
