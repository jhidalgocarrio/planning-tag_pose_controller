#include <tag_pose_controller/PID.h>

PID::PID()
{
  params_.Kp = 0.01;
  params_.Ki = 0.01;
  params_.Kd = 0.0;
  params_.upper_limit = 1.0;
  params_.lower_limit = -1.0;
  params_.scaling_factor = 1.0;
  params_.sampling_time_secs = 0.1; //in seconds
  error_sum = 0.0;
  control_output = 0.0;
}

PID::PID(const Parameters params)
{
  params_.Kp = params.Kp;
  params_.Ki = params.Ki;
  params_.Kd = params.Kd;
  params_.upper_limit = params.upper_limit;
  params_.lower_limit = params.lower_limit;
  params_.scaling_factor = params.scaling_factor;
  params_.sampling_time_secs = params.sampling_time_secs; //in seconds

}

void PID::setPidParams(double Kp, double Ki, double Kd,
                       double upper, double lower, double sampling_time,
                       double scaling_factor)
{
  params_.Kp = Kp;
  params_.Ki = Ki;
  params_.Kd = Kd;
  params_.upper_limit = upper;
  params_.lower_limit = lower;
  params_.sampling_time_secs = sampling_time;
  params_.scaling_factor = scaling_factor;
}

PID::~PID(){}

double PID::getControlSignal(const double& setpoint, const double& measured){

  double error = measured - setpoint;

  double error_gradiant = (error - last_error)/params_.sampling_time_secs;

  control_output = params_.Kp * error + params_.Ki * error_sum * params_.sampling_time_secs + params_.Kd * error_gradiant;

  if (control_output >= params_.upper_limit)
  {
    control_output = params_.upper_limit;
  }
  else if (control_output <= params_.lower_limit)
  {
    control_output = params_.lower_limit;
  }
  else
  {
    error_sum += error;
    last_error = error;
  }

}

void PID::debugOutput()
{
  printf("Kp: %f \n Ki: %f \n Kd: %f \n upper_limit: %f \n lower_limit: %f \n sampling_time: %f \n scaling_factor: %f", params_.Kp, params_.Ki, params_.Kd,
                      params_.upper_limit, params_.lower_limit, params_.sampling_time_secs, params_.scaling_factor);
}