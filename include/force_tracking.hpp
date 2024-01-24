#ifndef __FORCE_TRACKING
#define __FORCE_TRACKING

#include <iostream>
#include <ostream>
#include <fstream>
#include <Eigen/Dense>
#include <deque>

#include <leg_kinematics.hpp>

Eigen::Vector2d jointFriction(const Eigen::Vector2d &v_phi);
Eigen::Vector2d inverseDynamic(const std::deque<Eigen::Vector2d> &TB);
double stribeckFrictionModel(double v);

Eigen::Vector2d PositionBasedImpedanceFilter(const Eigen::Matrix2d &M, const Eigen::Matrix2d &K, const Eigen::Matrix2d &D,
                                             const Eigen::Matrix<double, 3, 2> &Xref, const Eigen::Matrix<double, 3, 2> &Fref,
                                             const Eigen::Matrix<double, 2, 2> &Xc, const Eigen::Matrix<double, 4, 2> &TB_fb,
                                             const Eigen::Matrix<double, 3, 2> &T_fb)
{
    /* Xref = [x_k, y_k;
              x_k_1, y_k_1;
              x_k_2, y_k_2;] */
    /* Xc = [xc_k_1, yc_k_1;
            yc_k_2, yc_k_2] */
    /* Fref = [Fx(k), Fy(k);
               Fx(k-1), Fy(k-1);
               Fx(k-2), Fy(k-2)] */
    /* TB_fb = [theta(k), beta(k);
                theta(k-1), beta(k-1);
                theta(k-2), beta(k-2)
                theta(k-3), beta(k-3)] */
    /* T_fb = [T_R(k), T_L(k);
               T_R(k-1), T_L(k-1);
               T_R(k-2), T_L(k-2)] */
    /* Fext: Force exert to ground (Obtain by Virtual work method)*/

    Eigen::Vector2d X_k = fk(TB_fb.row(0));
    Eigen::Vector2d X_k_1 = fk(TB_fb.row(1));
    Eigen::Vector2d X_k_2 = fk(TB_fb.row(2));

    Eigen::Vector2d tb_k = TB_fb.row(0);
    Eigen::Vector2d tb_k_1 = TB_fb.row(1);
    Eigen::Vector2d tb_k_2 = TB_fb.row(2);
    Eigen::Vector2d tb_k_3 = TB_fb.row(3);

    Eigen::Vector2d d_phi = dtb2dphi((tb_k - tb_k_1) / T_);
    Eigen::Vector2d d_phi_1 = dtb2dphi((tb_k_1 - tb_k_2) / T_);
    Eigen::Vector2d d_phi_2 = dtb2dphi((tb_k_2 - tb_k_3) / T_);
    Eigen::Vector2d tau_ft = jointFriction(d_phi);
    Eigen::Vector2d tau_ft_1 = jointFriction(d_phi_1);
    Eigen::Vector2d tau_ft_2 = jointFriction(d_phi_2);

    Eigen::Vector2d F_k = jointTrq2footendForce(T_fb.row(0).transpose() - tau_ft, TB_fb.row(0));
    Eigen::Vector2d F_k_1 = jointTrq2footendForce(T_fb.row(1).transpose() - tau_ft_1, TB_fb.row(1));
    Eigen::Vector2d F_k_2 = jointTrq2footendForce(T_fb.row(2).transpose() - tau_ft_2, TB_fb.row(2));

    // Eigen::Vector2d F_k = jointTrq2footendForce(T_fb.row(0).transpose(), TB_fb.row(0));
    // Eigen::Vector2d F_k_1 = jointTrq2footendForce(T_fb.row(1).transpose(), TB_fb.row(1));
    // Eigen::Vector2d F_k_2 = jointTrq2footendForce(T_fb.row(2).transpose(), TB_fb.row(2));

    Eigen::Vector2d d_F_k = F_k - Fref.row(0).transpose();
    Eigen::Vector2d d_F_k_1 = F_k_1 - Fref.row(1).transpose();
    Eigen::Vector2d d_F_k_2 = F_k_2 - Fref.row(2).transpose();

    /* Eigen::Vector2d d_F_k = F_k;
    Eigen::Vector2d d_F_k_1 = F_k_1;
    Eigen::Vector2d d_F_k_2 = F_k_2; */

    Eigen::Vector2d E_k_1 = Xref.row(1) - Xc.row(0);
    Eigen::Vector2d E_k_2 = Xref.row(2) - Xc.row(1);

    Eigen::Matrix<double, 2, 2> w1 = K * pow(T_, 2) + 2 * D * T_ + 4 * M;
    Eigen::Matrix<double, 2, 2> w2 = 2 * K * pow(T_, 2) - 8 * M;
    Eigen::Matrix<double, 2, 2> w3 = K * pow(T_, 2) - 2 * D * T_ + 4 * M;

    Eigen::Vector2d E_k;
    E_k = w1.inverse() * (pow(T_, 2) * (d_F_k + 2 * d_F_k_1 + d_F_k_2) - w2 * E_k_1 - w3 * E_k_2);
    Eigen::Vector2d Xc_k = Xref.row(0).transpose() - E_k;
    Eigen::Vector2d X_d = Xref.row(0);
    // return X_d;
    return Xc_k;
}

Eigen::Vector2d PositionBasedImpFilter(const Eigen::Matrix2d &M, const Eigen::Matrix2d &K, const Eigen::Matrix2d &D,
                                       const std::deque<Eigen::Vector2d> &Xref, const std::deque<Eigen::Vector2d> &Fref,
                                       const std::deque<Eigen::Vector2d> &Xc, const std::deque<Eigen::Vector2d> &TB_fb,
                                       const std::deque<Eigen::Vector2d> &T_fb)
{
    /* Xref = [x_k, y_k;
              x_k_1, y_k_1;
              x_k_2, y_k_2;] */
    /* Xc = [xc_k_1, yc_k_1;
            yc_k_2, yc_k_2] */
    /* Fref = [Fx(k), Fy(k);
               Fx(k-1), Fy(k-1);
               Fx(k-2), Fy(k-2)] */
    /* TB_fb = [theta(k), beta(k);
                theta(k-1), beta(k-1);
                theta(k-2), beta(k-2)
                theta(k-3), beta(k-3)] */
    /* T_fb = [T_R(k), T_L(k);
               T_R(k-1), T_L(k-1);
               T_R(k-2), T_L(k-2)] */
    /* Fext: Force exert to ground (Obtain by Virtual work method)*/

    Eigen::Vector2d X_k = fk(TB_fb[0]);
    Eigen::Vector2d X_k_1 = fk(TB_fb[1]);
    Eigen::Vector2d X_k_2 = fk(TB_fb[2]);

    Eigen::Vector2d d_phi = dtb2dphi((TB_fb[0] - TB_fb[1]) / T_);
    Eigen::Vector2d d_phi_1 = dtb2dphi((TB_fb[1] - TB_fb[2]) / T_);
    Eigen::Vector2d d_phi_2 = dtb2dphi((TB_fb[2] - TB_fb[3]) / T_);
    Eigen::Vector2d tau_ft = jointFriction(d_phi);
    Eigen::Vector2d tau_ft_1 = jointFriction(d_phi_1);
    Eigen::Vector2d tau_ft_2 = jointFriction(d_phi_2);

    Eigen::Vector2d F_k = jointTrq2footendForce(T_fb[0] - tau_ft, TB_fb[0]);
    Eigen::Vector2d F_k_1 = jointTrq2footendForce(T_fb[1] - tau_ft_1, TB_fb[1]);
    Eigen::Vector2d F_k_2 = jointTrq2footendForce(T_fb[2] - tau_ft_2, TB_fb[2]);

    /* Eigen::Vector2d d_F_k = F_k - Fref[0];
    Eigen::Vector2d d_F_k_1 = F_k_1 - Fref[1];
    Eigen::Vector2d d_F_k_2 = F_k_2 - Fref[2]; */

    Eigen::Vector2d d_F_k = F_k;
    Eigen::Vector2d d_F_k_1 = F_k_1;
    Eigen::Vector2d d_F_k_2 = F_k_2;

    Eigen::Vector2d E_k_1 = Xref[1] - Xc[0];
    Eigen::Vector2d E_k_2 = Xref[2] - Xc[1];

    Eigen::Matrix<double, 2, 2> w1 = K * pow(T_, 2) + 2 * D * T_ + 4 * M;
    Eigen::Matrix<double, 2, 2> w2 = 2 * K * pow(T_, 2) - 8 * M;
    Eigen::Matrix<double, 2, 2> w3 = K * pow(T_, 2) - 2 * D * T_ + 4 * M;

    Eigen::Vector2d E_k;
    E_k = w1.inverse() * (pow(T_, 2) * (d_F_k + 2 * d_F_k_1 + d_F_k_2) - w2 * E_k_1 - w3 * E_k_2);
    Eigen::Vector2d Xc_k = Xref[0] - E_k;

    return Xc_k;
}

Eigen::Vector2d forceEstimation(const Eigen::Vector2d &T_fb, const std::deque<Eigen::Vector2d> &TB_fb)
{
    // Return force exert to ground
    Eigen::Vector2d tau_inertia = inverseDynamic(TB_fb);
    Eigen::Vector2d d_phi = dtb2dphi((TB_fb[0] - TB_fb[1]) / T_);
    Eigen::Vector2d tau_friction = jointFriction(d_phi);
    Eigen::Vector2d F_est = jointTrq2footendForce(T_fb - tau_inertia - tau_friction, TB_fb[0]);
    return F_est;
}

Eigen::Vector2d inverseDynamic(const std::deque<Eigen::Vector2d> &TB)
{
    Eigen::Vector2d tb = TB[0];
    Eigen::Vector2d dtb = (TB[0] - TB[1]) / T_;
    Eigen::Vector2d dtb_1 = (TB[1] - TB[2]) / T_;
    Eigen::Vector2d ddtb = (dtb - dtb_1) / T_;

    /* q = [Rm; beta] */
    Eigen::Vector2d q(Rm(tb[0]), tb[1]);
    Eigen::Vector2d dq(dRm(tb[0], dtb[0]), dtb[1]);
    Eigen::Vector2d ddq(ddRm(tb[0], dtb[0], ddtb[0]), ddtb[1]);
    Eigen::Matrix2d Mq;
    Eigen::Vector2d Cq;
    Eigen::Vector2d Gq;
    Mq << leg_m, 0,
        0, Ic(tb[0]) + leg_m * pow(q[0], 2);
    Cq << -leg_m * q[0] * pow(q[1], 2),
        2 * leg_m * q[0] * dq[0] * dq[1] + dIc(tb[0], dtb[0]) * dq[1];
    Gq << -leg_m * g * cos(q[1]),
        -leg_m * g * q[0] * sin(q[1]);
    Eigen::Vector2d Frm_Tb;
    Eigen::Vector2d joint_trq;
    Frm_Tb = Mq * ddq + Cq + Gq;
    joint_trq = FrmTb2jointTrq(Frm_Tb, tb[0]);
    return joint_trq;
}

Eigen::Vector2d adaptiveStiffness(const Eigen::Vector2d &F_err,
                                  const std::deque<Eigen::Vector2d> &pid_out, const std::deque<Eigen::Vector2d> &pid_err,
                                  const Eigen::Vector2d &kp, const Eigen::Vector2d &ki, const Eigen::Vector2d &kd)
{
    // F_err = F_leg2gnd_ref - F_leg2gnd_est (GRF error)
    Eigen::Vector2d k_stiffness(0, 0);
    for (int i = 0; i < 2; i++)
    {
        double p_ = kp[i];
        double i_ = ki[i];
        double d_ = kd[i];
        double E_k = pid_err[0][i];
        double E_k_1 = pid_err[1][i];
        double E_k_2 = pid_err[2][i];
        double Y_k_2 = pid_out[2][i];
        double C2 = p_ + (i_ * T_ / 2) + (2 * d_ / T_);
        double C1 = i * T_ - 4 * d_ / T_;
        double C0 = -p_ + i_ * T_ / 2 + 2 * d_ / T_;
        double Y_k = C2 * E_k + C1 * E_k_1 + C0 * E_k_2;
        k_stiffness[i] = Y_k;
    }
    return k_stiffness;
}

Eigen::Vector2d jointFriction(const Eigen::Vector2d &v_phi)
{
    /* Apply Stribeck friction model */
    double tf_R = stribeckFrictionModel(v_phi[0]);
    double tf_L = stribeckFrictionModel(v_phi[1]);
    Eigen::Vector2d t_friction(tf_R, tf_L);
    return t_friction;
}

double stribeckFrictionModel(double v)
{
    double v_st = breakaway_vel * sqrt(2);
    double v_coul = breakaway_vel / 10;
    double e = std::exp(1);
    double F = sqrt(2 * e) * (breakaway_Ft - coulumb_Ft) * std::exp(-pow((v / v_st), 2)) * v / v_st + coulumb_Ft * tanh(v / v_coul) + viscous_cff * v;
    return F;
}

#endif