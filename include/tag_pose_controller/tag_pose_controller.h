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
    void calculatePidOutput(double& control_signal, const double desired, const double measured);
    void setParams(double Kp, double Ki, double Kd,
                    double upper, double lower, double sampling_time,
                      double scaling_factor);
private:

    PID pid;
    double distance, roll, pitch, yaw, control_signal;
    Eigen::Vector3d tag_in_base_pos;
    Eigen::Vector3d tag_in_base_rot;

};


#endif
