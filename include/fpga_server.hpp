// #include <leg_module.hpp>
#include <console.hpp>
#include <yaml.h>
#include <fpga_handler.hpp>
#include <fsm.hpp>
#include <msg.hpp>
#include <string>
#include <vector>
#include <brlos/NodeHandler.hpp>
#include <brlos/parameters_parser.hpp>
#include <mutex>

#include "boost/bind.hpp"
#include "boost/thread.hpp"

#define CONFIG_PATH "/home/admin/corgi_ws/src/fpga_server/config/config.yaml"

volatile sig_atomic_t sys_stop;
void inthand(int signum);
void cmdCallback(FpgaCmdMsg cmd_msg);

class Corgi
{
public:
  Corgi();
  void load_config_();

  YAML::Node yaml_node_;
  int modules_num_;

  FpgaHandler fpga_;
  Behavior behavior_;
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

  void interruptHandler(Subscriber &cmd_sub_, Publisher &state_pub_);
  void interruptHandler();

  void mainLoop_(Subscriber &cmd_sub_, Publisher &state_pub_);
  void mainLoop_();
  void canLoop_();
};
