#ifndef __FORCE_TRACKING
#define __FORCE_TRACKING

#include <iostream>
#include <ostream>
#include <fstream>
#include <Eigen/Dense>
#include <deque>

#include <leg_kinematics.hpp>

Eigen::Vector2d PositionBasedImpFilter(const Eigen::Matrix2d &M, const Eigen::Matrix2d &K, const Eigen::Matrix2d &D,
                                       const std::deque<Eigen::Vector2d> &Xref, const std::deque<Eigen::Vector2d> &Fref,
                                       const std::deque<Eigen::Vector2d> &Xc, const std::deque<Eigen::Vector2d> &TB_fb,
                                       const std::deque<Eigen::Vector2d> &T_fb, const std::deque<Eigen::Vector2d> &F_fb);

Eigen::Vector2d forceEstimation(const Eigen::Vector2d &T_fb, const std::deque<Eigen::Vector2d> &TB_fb, const Eigen::Vector2d &tau_friction);
Eigen::Vector2d forceEstimation(const Eigen::Vector2d &T_fb, const Eigen::Vector2d &TB, const Eigen::Vector2d &dTB,
                                const Eigen::Vector2d &ddTB, const Eigen::Vector2d &tau_friction);

Eigen::Vector2d inverseDynamic(const std::deque<Eigen::Vector2d> &TB);
// Inverse Dynamic with filtered input
Eigen::Vector2d inverseDynamic(const Eigen::Vector2d &TB,
                               const Eigen::Vector2d &dTB,
                               const Eigen::Vector2d &ddTB);

Eigen::Vector2d adaptiveStiffness(const Eigen::Vector2d &F_err,
                                  const std::deque<Eigen::Vector2d> &pid_out, const std::deque<Eigen::Vector2d> &pid_err,
                                  const Eigen::Vector2d &kp, const Eigen::Vector2d &ki, const Eigen::Vector2d &kd);

Eigen::Vector2d jointFriction(const Eigen::Vector2d &v_phi);

double stribeckFrictionModel(double v);

#endif
