#ifndef __FORCE_CONTROL
#define __FORCE_CONTROL

#include <iostream>
#include "force_tracking.hpp"
#include "lpf.hpp"

class ForceTracker
{
public:
    ForceTracker(Eigen::Matrix2d M, Eigen::Matrix2d K, Eigen::Matrix2d D,
                 Eigen::Vector2d a_kp, Eigen::Vector2d a_ki, Eigen::Vector2d a_kd);
    ForceTracker()
    {
    }

    Eigen::Vector2d init_tb;

    Eigen::Matrix2d M_d;
    Eigen::Matrix2d K_0;
    Eigen::Matrix2d D_d;

    Eigen::Matrix2d K_ft; // stiffness while applying adaptive law

    std::deque<Eigen::Vector2d> X_d_q;
    std::deque<Eigen::Vector2d> X_c_q;
    std::deque<Eigen::Vector2d> F_d_q;
    std::deque<Eigen::Vector2d> TB_fb_q;
    std::deque<Eigen::Vector2d> T_fb_q;
    std::deque<Eigen::Vector2d> F_fb_q;

    std::deque<Eigen::Vector2d> adaptive_pid_out;
    std::deque<Eigen::Vector2d> adaptive_pid_err;
    Eigen::Vector2d adaptive_kp;
    Eigen::Vector2d adaptive_ki;
    Eigen::Vector2d adaptive_kd;

    // friction model param.
    std::vector<double> breakaway_Ft_;
    std::vector<double> breakaway_vel_;
    std::vector<double> coulumb_Ft_;
    std::vector<double> viscous_cff_;

    template <typename T>
    void update_delay_state(std::deque<T> &state, T x);

    void initialize(const Eigen::Vector2d &init_tb);

    lowpassFilter trq_lpf_r;
    lowpassFilter trq_lpf_l;
    lowpassFilter vel_lpf_r;
    lowpassFilter vel_lpf_l;

    // Track desired force F_d and reference trajectory X_d (leg_frame)
    Eigen::Vector2d track(const Eigen::Vector2d &X_d, const Eigen::Vector2d &F_d,
                          const Eigen::Matrix2d &K_adapt);
    Eigen::Vector2d controlLoop(const Eigen::Vector2d &X_d, const Eigen::Vector2d &F_d,
                                const Eigen::Vector2d &tb_fb, const Eigen::Vector2d &trq_fb, const Eigen::Vector2d &phi_vel);

    Eigen::Vector2d jointFriction(const Eigen::Vector2d &v_phi);

    double stribeckFrictionModel(int idx, double v);
};

#endif
