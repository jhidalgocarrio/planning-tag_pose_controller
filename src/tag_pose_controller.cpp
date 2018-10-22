#include <tag_pose_controller/tag_pose_controller.h>

TagPoseController::TagPoseController()
{}

TagPoseController::~TagPoseController()
{}

const void TagPoseController::setLastDetectedPose(const Eigen::Vector3d& position, const Eigen::Vector3d& rotation)

{
  tag_in_base_pos = position;
  tag_in_base_rot = rotation;
}

void TagPoseController::calculateDistance()
{
  distance = sqrt(pow(tag_in_base_pos.x(),2) + pow(tag_in_base_pos.y(),2)
                         + pow(tag_in_base_pos.z(),2));
}

const double TagPoseController::getDistance()
{
  return distance;
}

void TagPoseController::calculatePidOutput(double& control_signal, const double setpoint, const double measured)
{
  control_signal = pid.getControlSignal(setpoint, measured);
}

void TagPoseController::setParams(double Kp, double Ki, double Kd,
                                  double upper, double lower, double sampling_time,
                                  double scaling_factor)
{
  pid.setPidParams(Kp, Ki, Kd, upper, lower, sampling_time, scaling_factor);
  pid.debugOutput();
}