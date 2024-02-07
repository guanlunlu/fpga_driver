#ifndef __LEG_KINEMATICS
#define __LEG_KINEMATICS

#include <Eigen/Dense>
#include <vector>

#include <ostream>
#include <fstream>

double deg2rad(double deg);
double rad2deg(double rad);
double polyval(const Eigen::VectorXd &coeff, double x);
Eigen::VectorXd polyder(const Eigen::VectorXd &coeff);

void kinematics_setup();

Eigen::Vector2d phi2tb(const Eigen::Vector2d &phi);
Eigen::Vector2d tb2phi(const Eigen::Vector2d &tb);
Eigen::Vector2d dtb2dphi(const Eigen::Vector2d &dtb);
Eigen::Vector2d dphi2dtb(const Eigen::Vector2d &dphi);

Eigen::Vector2d fk(const Eigen::Vector2d &tb);
Eigen::Vector2d ik(const Eigen::Vector2d &xy);
Eigen::Matrix2d jacG(const Eigen::Vector2d &tb);
Eigen::Matrix2d djacG(const Eigen::Vector2d &tb, const Eigen::Vector2d &dtb);
std::vector<Eigen::Vector2d> joint2footend_transform(const Eigen::Vector2d &q, const Eigen::Vector2d &q_dot);
Eigen::Matrix2d n_jacG(const Eigen::Vector2d &tb);
Eigen::Vector2d jointTrq2footendForce(const Eigen::Vector2d &joint_tau, const Eigen::Vector2d &tb);
Eigen::Vector2d FrmTb2jointTrq(const Eigen::Vector2d &FrmTb, double theta);
double Rm(const double &theta);
double Ic(const double &theta);
double dRm(const double &theta, const double &dtheta);
double ddRm(const double &theta, const double &dtheta, const double &ddtheta);
double dIc(const double &theta, const double &dtheta);
double polyval(const Eigen::VectorXd &coeff, double x);

Eigen::VectorXd polyder(const Eigen::VectorXd &coeff);
double deg2rad(double deg);
double rad2deg(double rad);

/* Polynomial fitting coeff. [fk: L_OG(theta) ik: theta(L_OG)] */
extern Eigen::VectorXd fk_pc_;
extern Eigen::VectorXd d_fk_pc_;
extern Eigen::VectorXd dd_fk_pc_;
extern Eigen::VectorXd ik_pc_;
extern Eigen::VectorXd rm_coeff;
extern Eigen::VectorXd d_rm_coeff;
extern Eigen::VectorXd dd_rm_coeff;
extern Eigen::VectorXd Ic_coeff;
extern Eigen::VectorXd d_Ic_coeff;
extern std::vector<int> a;

extern double T_;
extern double leg_m;
extern double g;
extern double breakaway_Ft;
extern double breakaway_vel;
extern double coulumb_Ft;
extern double viscous_cff;

extern std::ofstream term;

#endif