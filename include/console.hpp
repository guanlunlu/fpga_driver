#ifndef __CONSOLE_H
#define __CONSOLE_H

#define BKGD_PAIR 1
#define CYAN_PAIR 2
#define NCURSES_NOMACROS
#include <leg_module.hpp>
#include <fsm.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <algorithm>
#include <iomanip>
#include <unistd.h>
#include <ncurses.h>
#include <locale.h>
#include <mode.hpp>
using namespace std;

class Panel
{
public:
  Panel(string title, string type, LegModule *lm_, int org_x, int org_y, int height_, int width_, bool box_on);
  Panel()
  {
  }

  WINDOW *win_;

  string title_;

  /* module, control or debug*/
  string type_;
  int org_x_;
  int org_y_;
  int height_;
  int width_;
  bool box_on_;

  LegModule *md_ptr_;
  std::mutex *main_mtx_;
  std::vector<bool> *powerboard_state_;
  ModeFsm *fsm_;

  mutex mutex_;
  void infoDisplay();
  void infoDisplay(FpgaHandler *fpga, bool power_switch, bool signal_switch, bool digital_switch);
  void infoDisplay(Behavior bhv, Mode fsm_mode);
  void resetPanel();
  void panelTitle();
};

class InputPanel
{
public:
  InputPanel()
  {
  }

  void init(vector<LegModule> *mods_, bool *if_resetPanel, int term_max_x, int term_max_y);

  void inputHandler(WINDOW *win_, std::mutex &input_mutex);
  void reset_input_window(WINDOW *win);
  void commandDecode(std::string buf);
  vector<string> tokenizer(std::string s);
  auto getValue(string str);

  WINDOW *win_;
  std::mutex mutex_;

  LegModule *modA_ptr_;
  LegModule *modB_ptr_;
  LegModule *modC_ptr_;
  LegModule *modD_ptr_;

  bool *if_resetPanel;
  std::mutex *main_mtx_;
  std::vector<bool> *powerboard_state_;
  ModeFsm *fsm_;

private:
  std::thread *thread;
};

class Console
{
public:
  // Console(vector<LegModule>* mods_, std::vector<bool>* powerboard_state_, );
  Console()
  {
  }

  void init(FpgaHandler *fpga_, vector<LegModule> *mods_, std::vector<bool> *pb_state_, ModeFsm *fsm_, std::mutex *mtx_);
  void refreshWindow();

  int term_max_x_;
  int term_max_y_;
  int debug_cons_h = 27;
  int power_cons_h = 27;

  FpgaHandler *fpga_;

  Panel p_cmain_;
  Panel p_debug_;
  Panel p_mod1_;
  Panel p_mod2_;
  Panel p_mod3_;
  Panel p_mod4_;
  Panel p_control_;
  InputPanel input_panel_;

  LegModule *modA_ptr_;
  LegModule *modB_ptr_;
  LegModule *modC_ptr_;
  LegModule *modD_ptr_;

  std::mutex *main_mtx_;
  std::vector<bool> *powerboard_state_;
  ModeFsm *fsm_;

  mutex input_mutex_;
  thread t_frontend_;
  int frontend_rate_;

  bool if_resetPanel;
};

#endif
