#include <force_control.hpp>

ForceTracker::ForceTracker(Eigen::Matrix2d M, Eigen::Matrix2d K, Eigen::Matrix2d D,
                           Eigen::Vector2d a_kp, Eigen::Vector2d a_ki, Eigen::Vector2d a_kd)
{
    M_d = M;
    K_0 = K;
    D_d = D;
    adaptive_kp = a_kp;
    adaptive_kp = a_ki;
    adaptive_kp = a_kd;

    trq_lpf_r.init(10, T_);
    trq_lpf_l.init(10, T_);
    vel_lpf_r.init(20, T_);
    vel_lpf_l.init(20, T_);
}

void ForceTracker::initialize(const Eigen::Vector2d &init_tb)
{
    // initialize state queue
    Eigen::Vector2d z(0, 0);
    Eigen::Vector2d init_xy = fk(init_tb);

    X_d_q.push_front(init_xy);
    X_d_q.push_front(init_xy);
    X_d_q.push_front(init_xy);

    F_d_q.push_front(z);
    F_d_q.push_front(z);
    F_d_q.push_front(z);

    X_c_q.push_front(init_xy);
    X_c_q.push_front(init_xy);

    TB_fb_q.push_front(init_tb);
    TB_fb_q.push_front(init_tb);
    TB_fb_q.push_front(init_tb);
    TB_fb_q.push_front(init_tb);

    T_fb_q.push_front(z);
    T_fb_q.push_front(z);
    T_fb_q.push_front(z);

    F_fb_q.push_front(z);
    F_fb_q.push_front(z);
    F_fb_q.push_front(z);

    adaptive_pid_out.push_front(z);
    adaptive_pid_out.push_front(z);
    adaptive_pid_out.push_front(z);
    adaptive_pid_err.push_front(z);
    adaptive_pid_err.push_front(z);
    adaptive_pid_err.push_front(z);
}

Eigen::Vector2d ForceTracker::track(const Eigen::Vector2d &X_d, const Eigen::Vector2d &F_d,
                                    const Eigen::Matrix2d &K_adapt)
{
    // Eigen::Matrix2d K_d = K_0 + K_adapt;
    Eigen::Matrix2d K_d = K_0;
    Eigen::Vector2d Xc_k = PositionBasedImpFilter(M_d, K_d, D_d, X_d_q, F_d_q, X_c_q, TB_fb_q, T_fb_q, F_fb_q);
    update_delay_state<Eigen::Vector2d>(X_c_q, Xc_k);

    Eigen::Vector2d tb_k = ik(Xc_k);
    Eigen::Vector2d phi_k = tb2phi(tb_k);

    return phi_k;
}

Eigen::Vector2d ForceTracker::controlLoop(const Eigen::Vector2d &X_d, const Eigen::Vector2d &F_d,
                                          const Eigen::Vector2d &tb_fb, const Eigen::Vector2d &trq_fb_filt, const Eigen::Vector2d &phi_vel_fb_filt)
{
    update_delay_state<Eigen::Vector2d>(TB_fb_q, tb_fb);
    update_delay_state<Eigen::Vector2d>(T_fb_q, trq_fb_filt);
    update_delay_state<Eigen::Vector2d>(X_d_q, X_d);
    update_delay_state<Eigen::Vector2d>(F_d_q, F_d);

    term << phi_vel_fb_filt[0] << "," << phi_vel_fb_filt[1] << ",";

    Eigen::Vector2d tau_friction = jointFriction(phi_vel_fb_filt);
    Eigen::Vector2d F_est_l2g = forceEstimation(trq_fb_filt, TB_fb_q, tau_friction);
    Eigen::Vector2d F_est_g2l = -1 * F_est_l2g;
    Eigen::Vector2d F_err_g2l = F_d - F_est_g2l;
    update_delay_state<Eigen::Vector2d>(F_fb_q, F_est_l2g);

    term << F_est_l2g[0] << "," << F_est_l2g[1] << ",";

    // adaptive stiffness
    update_delay_state<Eigen::Vector2d>(adaptive_pid_err, F_err_g2l);
    Eigen::Vector2d K_adapt = adaptiveStiffness(F_err_g2l, adaptive_pid_out, adaptive_pid_err, adaptive_kp, adaptive_ki, adaptive_kd);
    update_delay_state<Eigen::Vector2d>(adaptive_pid_out, K_adapt);
    Eigen::Matrix2d K_adpt;
    K_adpt << K_adapt[0], 0, 0, K_adapt[1];

    Eigen::Vector2d phi = track(X_d, F_d, K_adpt);

    return phi;
}

Eigen::Vector2d ForceTracker::jointFriction(const Eigen::Vector2d &v_phi)
{
    double tf_R = stribeckFrictionModel(0, v_phi[0]);
    double tf_L = stribeckFrictionModel(1, v_phi[1]);
    Eigen::Vector2d t_friction(tf_R, tf_L);
    return t_friction;
}

double ForceTracker::stribeckFrictionModel(int idx, double v)
{
    double v_st = breakaway_vel_[idx] * sqrt(2);
    double v_coul = breakaway_vel_[idx] / 10;
    double e = std::exp(1);
    double F = sqrt(2 * e) * (breakaway_Ft_[idx] - coulumb_Ft_[idx]) * std::exp(-pow((v / v_st), 2)) * v / v_st + coulumb_Ft_[idx] * tanh(v / v_coul) + viscous_cff_[idx] * v;
    return F;
}

template <typename T>
void ForceTracker::update_delay_state(std::deque<T> &state, T x)
{
    state.push_front(x);
    state.pop_back();
}