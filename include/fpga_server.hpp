// #include <leg_module.hpp>
#include <console.hpp>
#include <yaml.h>
#include <fpga_handler.hpp>
#include <fsm.hpp>
#include <msg.hpp>
#include <string>
#include <vector>
#include <NodeHandler.hpp>
#include <mutex>
#include "motor_msg.hpp"

#include "boost/bind.hpp"
#include "boost/thread.hpp"

#define CONFIG_PATH "/home/admin/fpga_driver/config/config.yaml"

volatile sig_atomic_t sys_stop;
void inthand(int signum);

class Corgi
{
public:
  Corgi();
  void load_config_();

  YAML::Node yaml_node_;
  int modules_num_;

  FpgaHandler fpga_;
  Console console_;

  ModeFsm fsm_;
  std::vector<LegModule> modules_list_;
  std::vector<bool> powerboard_state_;
  std::mutex main_mtx_;

  int main_irq_period_us_;
  int can_irq_period_us_;

  bool digital_switch_;
  bool signal_switch_;
  bool power_switch_;
  bool stop_;

  void interruptHandler(std::vector<core::Subscriber> &cmd_sub_, std::vector<core::Publisher> &state_pub_);

  void mainLoop_(std::vector<core::Subscriber> &cmd_sub_, std::vector<core::Publisher> &state_pub_);
  void canLoop_();
};
