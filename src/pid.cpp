#include "pid.hpp"

PID_controller::PID_controller(double kp_, double ki_, double kd_, double T_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
    dT = T_;
    err = 0;
    prev_err = 0;
    err_sum = 0;
}

double PID_controller::tracking(double setpoint, double state)
{
    double output = 0;
    err = setpoint - state;
    err_sum += dT * err;
    double err_dev = (err - prev_err) / dT;
    prev_err = err;

    return kp * err + ki * err_sum + kd * err_dev;
}
