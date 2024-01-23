#ifndef __FSM_H
#define __FSM_H

#include <vector>
#include <leg_module.hpp>
#include <theta_error.hpp>
#include <fstream>
#include <unistd.h>

class ModeFsm
{
public:
  /* pass modules vector by reference*/
  ModeFsm(std::vector<LegModule> *module_list_, std::vector<bool> *pb_state_);
  ModeFsm()
  {
  }

  Mode workingMode_;
  Mode prev_workingMode_;

  std::vector<LegModule> *modules_list_;
  std::vector<bool> *pb_state_;

  bool hall_calibrated;
  int hall_calibrate_status;

  int measure_offset = 0;
  double dt_ = 0.001;  // second
  double cal_vel_ = 0.25; // rad/s
  double cal_tol_ = 0.05;
  double cal_dir_[4][2];
  double cal_command[4][2];

  void runFsm();
  bool switchMode(Mode next_mode);
};

#endif