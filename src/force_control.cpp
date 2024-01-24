#include <force_control.hpp>

ForceTracker::ForceTracker(Eigen::Vector2d init_xy, Eigen::Vector2d a_kp, Eigen::Vector2d a_ki, Eigen::Vector2d a_kd)
{
    // initialize state queue
    Eigen::Vector2d z(0, 0);
    init_tb = ik(init_xy);
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

    adaptive_kp = a_kp;
    adaptive_kp = a_ki;
    adaptive_kp = a_kd;
    adaptive_pid_out.push_front(z);
    adaptive_pid_out.push_front(z);
    adaptive_pid_out.push_front(z);
    adaptive_pid_err.push_front(z);
    adaptive_pid_err.push_front(z);
    adaptive_pid_err.push_front(z);

    M_d << 0.652, 0,
        0, 0.652;
    K_d << 50000, 0,
        0, 50000;
    D_d << 400, 0,
        0, 400;
}

Eigen::Vector2d ForceTracker::track(const Eigen::Vector2d &X_d, const Eigen::Vector2d &F_d, const Eigen::Matrix2d &K_adapt)
{
    Eigen::Vector2d Xc_k = PositionBasedImpFilter(M_d, K_d, D_d, X_d_q, F_d_q, X_c_q, TB_fb_q, T_fb_q);
    update_delay_state<Eigen::Vector2d>(X_c_q, Xc_k);
    Eigen::Vector2d tb_k = ik(Xc_k);
    Eigen::Vector2d phi_k = tb2phi(tb_k);
    return phi_k;
}

Eigen::Vector2d ForceTracker::controlLoop(const Eigen::Vector2d &X_d, const Eigen::Vector2d &F_d, const Eigen::Vector2d &tb_fb, const Eigen::Vector2d &trq_fb)
{
    update_delay_state<Eigen::Vector2d>(TB_fb_q, tb_fb);
    update_delay_state<Eigen::Vector2d>(T_fb_q, trq_fb);
    update_delay_state<Eigen::Vector2d>(X_d_q, X_d);
    update_delay_state<Eigen::Vector2d>(F_d_q, F_d);

    Eigen::Vector2d F_est_l2g = forceEstimation(trq_fb, TB_fb_q);
    Eigen::Vector2d F_est_g2l = -1 * F_est_l2g;
    Eigen::Vector2d F_err_g2l = F_d - F_est_g2l;

    update_delay_state<Eigen::Vector2d>(adaptive_pid_err, F_err_g2l);
    Eigen::Vector2d K_adapt = adaptiveStiffness(F_err_g2l, adaptive_pid_out, adaptive_pid_err, adaptive_kp, adaptive_ki, adaptive_kd);
    update_delay_state<Eigen::Vector2d>(adaptive_pid_out, K_adapt);

    Eigen::Matrix2d K_adpt;
    K_adpt << K_adapt[0], 0, 0, K_adapt[1];
    Eigen::Vector2d phi = track(X_d, F_d, K_adpt);
    return phi;
}

template <typename T>
void ForceTracker::update_delay_state(std::deque<T> &state, T x)
{
    state.push_front(x);
    state.pop_back();
}