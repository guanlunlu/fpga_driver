#ifndef _PID_H
#define _PID_H

class PID_controller{
public:
    PID_controller(double kp, double ki, double kd, double T_);
    PID_controller()
    {
    }

    double tracking(double setpoint, double state);

    double dT;
    double kp;
    double ki;
    double kd;

    double err;
    double prev_err;
    double err_sum;

};


#endif // DEBUG
