#include <lpf.hpp>
#include "leg_kinematics.hpp"

lowpassFilter::lowpassFilter(double w_c, double T)
{
    u_k_1 = 0;
    y_k_1 = 0;
    y_k = 0;
    w_c_ = w_c;
    dT = T;
}

void lowpassFilter::init(double w_c, double T)
{
    w_c_ = w_c;
    dT = T;
}

void lowpassFilter::reset()
{
    u_k_1 = 0;
    y_k_1 = 0;
    y_k = 0;
}

double lowpassFilter::update(double u_k)
{
    // term << "u_k: " << u_k << std::endl;
    double c0 = dT * w_c_;
    double c1 = dT * w_c_;
    double c2 = dT * w_c_ + 2;
    double c3 = dT * w_c_ - 2;
    double y_k_ = (c0 * u_k + c1 * u_k_1 - c3 * y_k_1) / c2;
    // term << "dT: " << dT << ", "
    //      << "w_c_: " << w_c_ << std::endl;
    // term << "c: " << c0 << ", " << c1 << ", " << c2 << ", " << c3 << std::endl;
    // term << "y_k: " << y_k << std::endl;
    u_k_1 = u_k;
    y_k_1 = y_k_;
    y_k = y_k_;
    return y_k_;
}