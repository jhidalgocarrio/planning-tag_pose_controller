#include <tag_pose_controller/PID.h>

PID::PID()
{
  Kp = 0.01;
  Ki = 0.01;
  Kd = 0.0;
  upper_limit = 1.0;
  lower_limit = -1.0;
  scaling_factor = 1.0;
  sampling_time_secs = 0.1; //in seconds
  error_sum = 0.0;
  control_output = 0.0;
}

PID::PID(double Kp_gain, double Ki_gain, double Kd_gain, double upper, double lower, double scaling)

: Kp(Kp_gain), Ki(Ki_gain), Kd(Kd_gain), upper_limit(upper),
  lower_limit(lower), scaling_factor(scaling) {}

PID::~PID(){}

double PID::getControlSignal(double& setpoint, double& measured){

  double error = measured - setpoint;

  double error_gradiant = (error - last_error)/sampling_time_secs;

  control_output = Kp * error + Ki * error_sum * sampling_time_secs + Kd * error_gradiant;

  if (control_output >= upper_limit)
  {
    control_output = upper_limit;
  }
  else if (control_output <= lower_limit)
  {
    control_output = lower_limit;
  }
  else
  {
    error_sum += error;
    last_error = error;
  }

}
