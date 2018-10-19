#ifndef TAG_POSE_CONTROLLER_H
#define TAG_POSE_CONTROLLER_H

#include <tag_pose_controller/PID.h>
#include <eigen3/Eigen/Core>
class TagPoseController {

public:

    TagPoseController();
    ~TagPoseController();
    const void setLastDetectedPose(const Eigen::Vector3d& position,const Eigen::Vector3d& rotation);
    void calculateDistance();
    const double getDistance();

private:

    PID pid;
    double distance, roll, pitch, yaw;
    Eigen::Vector3d tag_in_base_pos;
    Eigen::Vector3d tag_in_base_rot;

};


#endif
