#ifndef __FSM_H
#define __FSM_H

#include "force_tracking.hpp"

#include <vector>
#include <leg_module.hpp>
#include <theta_error.hpp>
#include <fstream>
#include <unistd.h>

#include "case_enum.hpp"

#include "motor.pb.h"
#include "power.pb.h"

class ModeFsm
{
public:
  /* pass modules vector by reference*/
  ModeFsm(std::vector<LegModule> *module_list_, std::vector<bool> *pb_state_, double *pb_v);
  ModeFsm()
  {
  }

  Mode workingMode_;
  Mode prev_workingMode_;

  Scenario scenario_;
  Command_type cmd_type_;

  std::vector<LegModule> *modules_list_;
  std::vector<bool> *pb_state_;

  bool hall_calibrated;
  int hall_calibrate_status;

  int measure_offset = 0;
  double dt_ = 0.001;     // second
  double cal_vel_ = 0.25; // rad/s
  double cal_tol_ = 0.05;
  double cal_dir_[4][2];
  double cal_command[4][2];

  bool *NO_CAN_TIMEDOUT_ERROR_;
  bool *NO_SWITCH_TIMEDOUT_ERROR_;
  double *powerboard_voltage;

  void runFsm(motor_msg::MotorStamped &motor_fb_msg, motor_msg::MotorStamped &motor_cmd_msg);
  bool switchMode(Mode next_mode);
  void publishMsg(motor_msg::MotorStamped &motor_fb_msg);
};

#endif