#ifndef PID_H
#define PID_H



class PID {

public:
    PID();
    PID(double Kp_gain, double Ki_gain, double Kd_gain, double upper, double lower, double scaling);
    ~PID();
    double getControlSignal(double& setpoint, double& measured);

private:
    double Kp, Ki, Kd;
    double error_sum, last_error, sampling_time_secs;
    double upper_limit, lower_limit;
    double control_output;
    double scaling_factor;
};

#endif
