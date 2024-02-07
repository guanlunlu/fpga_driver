#include "leg_kinematics.hpp"

double T_;
double leg_m;
double g;
double breakaway_Ft;
double breakaway_vel;
double coulumb_Ft;
double viscous_cff;

Eigen::VectorXd fk_pc_(8);
Eigen::VectorXd d_fk_pc_(7);
Eigen::VectorXd dd_fk_pc_(6);
Eigen::VectorXd ik_pc_(8);
Eigen::VectorXd rm_coeff(5);
Eigen::VectorXd d_rm_coeff(4);
Eigen::VectorXd dd_rm_coeff(3);
Eigen::VectorXd Ic_coeff(5);
Eigen::VectorXd d_Ic_coeff(4);

void kinematics_setup()
{
    T_ = 0.001;
    /* Physics prop. (SI units)*/
    leg_m = 0.654;
    g = 9.80665;
    /* Kinematics model coeff. */
    fk_pc_ << -0.0004362, 0.00715474, -0.04529168, 0.13936602, -0.22479562, 0.19297458, 0.01846436, 0.08280706;
    d_fk_pc_ = polyder(fk_pc_);
    dd_fk_pc_ = polyder(d_fk_pc_);
    ik_pc_ << 4.10837754e+05, -6.25071622e+05, 4.02616603e+05, -1.41754291e+05, 2.93469460e+04, -3.56228405e+03, 2.44240485e+02, -7.12383946e+00;

    /* Dynamics model coeff. */
    rm_coeff << -0.0035, 0.0110, 0.0030, 0.0500, -0.0132;
    Ic_coeff << 0.0001, -0.0001, -0.0013, 0.0043, 0.0041;
    d_rm_coeff = polyder(rm_coeff);
    dd_rm_coeff = polyder(rm_coeff);
    d_Ic_coeff = polyder(Ic_coeff);

    breakaway_Ft = 0.44;
    breakaway_vel = 0.01;
    coulumb_Ft = 0.2;
    viscous_cff = 0.3;
}

Eigen::Vector2d phi2tb(const Eigen::Vector2d &phi)
{
    Eigen::Vector2d tb;
    /* Eigen::Matrix2d t;
    Eigen::Vector2d b;
    t << 1, -1, 1, 1;
    b << deg2rad(17), 0;
    tb = 1 / 2 * t * phi + b; */

    std::complex<double> r_cpx(0, phi[0] + deg2rad(17));
    std::complex<double> l_cpx(0, phi[1] - deg2rad(17));
    std::complex<double> drl = std::exp(r_cpx) / std::exp(l_cpx);
    double theta = std::arg(drl);
    if (theta < 0)
        theta += 2 * M_PI;
    theta *= 0.5;
    double beta = std::arg(std::exp(l_cpx)) + theta;
    tb << theta, beta;
    return tb;
}

Eigen::Vector2d tb2phi(const Eigen::Vector2d &tb)
{
    Eigen::Vector2d phi;
    Eigen::Matrix2d t;
    Eigen::Vector2d b;
    t << 1, 1, -1, 1;
    b << deg2rad(17), -deg2rad(17);
    phi = t * tb - b;
    return phi;
}

Eigen::Vector2d dtb2dphi(const Eigen::Vector2d &dtb)
{
    Eigen::Vector2d dphi;
    Eigen::Matrix2d t;
    t << 1, 1, -1, 1;
    dphi = t * dtb;
    return dphi;
}

Eigen::Vector2d dphi2dtb(const Eigen::Vector2d &dphi)
{
    Eigen::Vector2d dtb;
    Eigen::Matrix2d t;
    t << 0.5, -0.5, 0.5, 0.5;
    dtb = t * dphi;
    return dtb;
}

Eigen::Vector2d fk(const Eigen::Vector2d &tb)
{
    double theta = tb[0];
    double beta = tb[1];

    double L = polyval(fk_pc_, theta);
    Eigen::Vector2d point_G;
    point_G << -L * sin(beta), -L * cos(beta);
    return point_G;
}

Eigen::Vector2d ik(const Eigen::Vector2d &xy)
{
    double x = xy[0];
    double y = xy[1];

    double L = sqrt(pow(x, 2) + pow(y, 2));
    double theta = polyval(ik_pc_, L);

    Eigen::Vector2d u_xy;
    Eigen::Vector2d u_y(0, 1);
    u_xy = xy / L;
    double beta = acos(-u_xy.adjoint() * u_y);
    if (x > 0)
    {
        beta = fabs(beta) * -1;
    }
    else
    {
        beta = fabs(beta);
    }
    Eigen::Vector2d tb;
    tb << theta, beta;
    return tb;
}

Eigen::Matrix2d jacG(const Eigen::Vector2d &tb)
{
    /* Jacobian of foot end point G */
    double theta = tb[0];
    double beta = tb[1];

    double L_p_theta = polyval(d_fk_pc_, theta);
    double L = polyval(fk_pc_, theta);

    Eigen::Matrix2d jG;
    jG << -L_p_theta * sin(beta), -L * cos(beta),
        -L_p_theta * cos(beta), L * sin(beta);
    return jG;
}

Eigen::Matrix2d djacG(const Eigen::Vector2d &tb, const Eigen::Vector2d &dtb)
{
    double theta = tb[0];
    double beta = tb[1];
    double dtheta = dtb[0];
    double dbeta = dtb[1];

    double dj00 = -1 * (polyval(dd_fk_pc_, theta) * dtheta * sin(beta) + polyval(d_fk_pc_, theta) * cos(beta) * dbeta);
    double dj01 = -1 * (polyval(d_fk_pc_, theta) * dtheta * cos(beta) - polyval(fk_pc_, theta) * sin(beta) * dbeta);
    double dj10 = -1 * (polyval(dd_fk_pc_, theta) * dtheta * cos(beta) - polyval(d_fk_pc_, theta) * sin(beta) * dbeta);
    double dj11 = (polyval(d_fk_pc_, theta) * dtheta * sin(beta) + polyval(fk_pc_, theta) * cos(beta) * dbeta);
    Eigen::Matrix2d dj;
    dj << dj00, dj01,
        dj10, dj11;
    return dj;
}

std::vector<Eigen::Vector2d> joint2footend_transform(const Eigen::Vector2d &q, const Eigen::Vector2d &q_dot)
{
    /* Convert joint space full state to Cartesian space */
    /* q = [theta, beta], q_dot = [theta_dot, beta_dot] */
    /* return footend_State = [fe_pos, fe_vel] */

    std::vector<Eigen::Vector2d> footend_state;
    Eigen::Vector2d fe_pos;
    Eigen::Vector2d fe_vel;
    // Eigen::Vector2d fe_acc;
    fe_pos = fk(q);
    fe_vel = jacG(q) * q_dot;
    // fe_acc = jacG(q) * q_ddot + djacG(q, q_dot) * q_dot;
    footend_state.push_back(fe_pos);
    footend_state.push_back(fe_vel);
    // footend_state.push_back(fe_acc);
    return footend_state;
}

Eigen::Matrix2d n_jacG(const Eigen::Vector2d &tb)
{
    double d = 0.001;
    Eigen::Vector2d tb_tp(tb[0] + d, tb[1]);
    Eigen::Vector2d tb_tn(tb[0] - d, tb[1]);
    Eigen::Vector2d tb_bp(tb[0], tb[1] + d);
    Eigen::Vector2d tb_bn(tb[0], tb[1] - d);

    Eigen::Vector2d dX_dt = (fk(tb_tp) - fk(tb_tn)) / (2 * d);
    Eigen::Vector2d dX_db = (fk(tb_bp) - fk(tb_bn)) / (2 * d);

    Eigen::Matrix2d jG{{dX_dt[0], dX_db[0]},
                       {dX_dt[1], dX_db[1]}};
    return jG;
}

Eigen::Vector2d jointTrq2footendForce(const Eigen::Vector2d &joint_tau, const Eigen::Vector2d &tb)
{
    Eigen::Matrix2d J_phi{{0.5, -0.5},
                          {0.5, 0.5}};

    Eigen::Matrix2d J_theta = jacG(tb);
    Eigen::Matrix2d J_1_T = (J_theta * J_phi).inverse().transpose();

    return J_1_T * joint_tau;
}

Eigen::Vector2d FrmTb2jointTrq(const Eigen::Vector2d &FrmTb, double theta)
{
    Eigen::Matrix2d J_phi{{0.5, -0.5},
                          {0.5, 0.5}};
    Eigen::Matrix2d J_theta{{polyval(d_rm_coeff, theta), 0},
                            {0, 1}};
    Eigen::Vector2d joint_trq;
    joint_trq = J_phi.transpose() * J_theta.transpose() * FrmTb;

    // std::cout << "J_phi\n";
    // std::cout << J_phi.transpose() << std::endl;
    // std::cout << "J_theta\n";
    // std::cout << J_theta.transpose() << std::endl;
    // std::cout << "FrmTb\n";
    // std::cout << FrmTb << std::endl;

    return joint_trq;
    // return J_phi.transpose() * J_theta.transpose() * FrmTb;
}

double Rm(const double &theta)
{
    return polyval(rm_coeff, theta);
}

double Ic(const double &theta)
{
    return polyval(Ic_coeff, theta);
}

double dRm(const double &theta, const double &dtheta)
{
    return polyval(d_rm_coeff, theta) * dtheta;
}

double ddRm(const double &theta, const double &dtheta, const double &ddtheta)
{
    return polyval(d_rm_coeff, theta) * ddtheta + polyval(dd_rm_coeff, theta) * dtheta * dtheta;
}

double dIc(const double &theta, const double &dtheta)
{
    return polyval(d_Ic_coeff, theta) * dtheta;
}

/* Math operations */
double polyval(const Eigen::VectorXd &coeff, double x)
{
    double px = 0;
    int ord = coeff.size() - 1;

    for (int n = ord; n >= 0; n--)
    {
        px += coeff(coeff.size() - 1 - n) * pow(x, n);
    }
    return px;
}

Eigen::VectorXd polyder(const Eigen::VectorXd &coeff)
{
    int n = coeff.size() - 1;
    Eigen::VectorXd d_coeff(n);
    d_coeff.fill(0);

    for (int i = 0; i <= n - 1; i++)
    {
        d_coeff[i] = coeff[i] * (n - i);
    }
    return d_coeff;
}

double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}