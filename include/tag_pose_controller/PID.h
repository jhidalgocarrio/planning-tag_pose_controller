#ifndef PID_H
#define PID_H

#include <stdio.h>

struct Parameters
{
    double Kp, Ki, Kd;
    double upper_limit, lower_limit;
    double scaling_factor, sampling_time_secs;
};


class PID {

public:
    PID();
    PID(const Parameters params);
    ~PID();
    double getControlSignal(const double& setpoint, const double& measured);
    void setPidParams(double Kp, double Ki, double Kd,
                    double upper, double lower, double sampling_time,
                      double scaling_factor);
                    
    void debugOutput();
private:
    double error_sum, last_error;
    double control_output;
    Parameters params_;

};

#endif
