#include <console.hpp>
#include <curses.h>
#include <iterator>

using namespace std;

mutex cons_mtx_;

void Console::init(vector<LegModule> *mods_, std::vector<bool> *pb_state_ptr_, ModeFsm *fsm_ptr_, std::mutex *mtx_ptr_)
{
    modA_ptr_ = &mods_->at(0);
    modB_ptr_ = &mods_->at(1);
    modC_ptr_ = &mods_->at(2);
    modD_ptr_ = &mods_->at(3);

    setlocale(LC_ALL, "");
    initscr();
    getmaxyx(stdscr, term_max_y_, term_max_x_);
    start_color();
    init_pair(BKGD_PAIR, COLOR_WHITE, COLOR_BLACK);
    wbkgd(stdscr, COLOR_PAIR(BKGD_PAIR));
    init_pair(CYAN_PAIR, COLOR_CYAN, COLOR_BLACK);

    frontend_rate_ = 3;
    // InputPanel input_panel_(mdA_, mdB_, mdC_, mdD_, term_max_x_, term_max_y_);
    input_panel_.init(mods_, &if_resetPanel, term_max_x_, term_max_y_);

    input_panel_.main_mtx_ = mtx_ptr_;
    input_panel_.powerboard_state_ = pb_state_ptr_;
    input_panel_.fsm_ = fsm_ptr_;

    main_mtx_ = mtx_ptr_;
    powerboard_state_ = pb_state_ptr_;
    fsm_ = fsm_ptr_;

    if_resetPanel = false;
    t_frontend_ = thread(&Console::refreshWindow, this);
}

void Console::refreshWindow()
{
    clear();

    int refresh_period_ = (int)(1 / frontend_rate_) * 1000000;
    LegModule *lm_null = 0;
    Panel p_power_("[P] Power Board ", "power", lm_null, 1, 1, power_cons_h, 40, true);
    Panel p_cmain_("[F] FPGA Server ", "c_main", lm_null, 1, 9, 8, 40, true);

    Panel p_modA_("[A] LF_Module ", "module", modA_ptr_, 41, 1, (term_max_y_ - 2) / 2 - 1, 60, true);
    Panel p_modD_("[D] LH_Module ", "module", modD_ptr_, 41, (term_max_y_) / 2, (term_max_y_ - 2) / 2 - 1, 60, true);
    Panel p_modB_("[B] RF_Module ", "module", modB_ptr_, 101, 1, (term_max_y_ - 2) / 2 - 1, 60, true);
    Panel p_modC_("[C] RH_Module ", "module", modC_ptr_, 101, (term_max_y_) / 2, (term_max_y_ - 2) / 2 - 1, 60, true);

    /* Panel p_debug_("[D] Debug Console ", "debug", lm_null, 1, (term_max_y_ - 1) - debug_cons_h, debug_cons_h - 2, 40,
                   true); */

    p_power_.powerboard_state_ = powerboard_state_;
    p_cmain_.fsm_ = fsm_;
    refresh();

    while (1)
    {
        cons_mtx_.lock();

        p_power_.infoDisplay(powerboard_state_->at(0), powerboard_state_->at(1), powerboard_state_->at(2));
        p_cmain_.infoDisplay(Behavior::TCP_SLAVE, fsm_->workingMode_);
        p_modA_.infoDisplay();
        p_modB_.infoDisplay();
        p_modC_.infoDisplay();
        p_modD_.infoDisplay();
        // p_cmain_.infoDisplay(Behavior::TCP_SLAVE, Mode::MOTOR, 1);

        cons_mtx_.unlock();

        usleep(0.1 * 1000 * 1000);
    }
}

void InputPanel::init(vector<LegModule> *mods_, bool *if_resetPanel, int term_max_x, int term_max_y)
{
    win_ = newwin(3, term_max_x - 1, term_max_y - 3, 1);
    // box(win_, 0, 0);

    modA_ptr_ = &mods_->at(0);
    modB_ptr_ = &mods_->at(1);
    modC_ptr_ = &mods_->at(2);
    modD_ptr_ = &mods_->at(3);

    thread = new std::thread(&InputPanel::inputHandler, this, win_, std::ref(mutex_));
}

void InputPanel::inputHandler(WINDOW *win_, std::mutex &input_mutex)
{
    while (1)
    {
        int ch = 0;
        int x = 1;
        do
        {
            ch = mvwgetch(win_, 1, x);
            if (ch == 'r')
            {
                reset_input_window(win_);
            }

        } while (ch != ':');

        string input_buf;
        keypad(win_, true);

        do
        {
            /* char char_ = 0;
            char_ = wgetch(win_);

            if (char_ == KEY_ENTER)
                break;
            else if (char_ == KEY_BACKSPACE)
            {
                if (x > 0)
                {
                    mvwdelch(win_, 1, x);
                    mvwdelch(win_, 1, x + 1);
                    mvwdelch(win_, 1, x - 1);
                }
                if (input_buf.size() > 0)
                {
                    input_buf.erase(input_buf.size() - 1, 1);
                    x--;
                }
            }
            else
            {
                input_buf.append(1, char_);
                cmtx_.lock();
                // mvwprintw(win_, 1, x, input_buf.c_str());
                cmtx_.unlock();
                x++;
            } */

            ch = mvwgetch(win_, 1, x);

            if (ch == KEY_BACKSPACE || ch == KEY_DC || ch == 127)
            {
                mvwdelch(win_, 1, x);
                mvwdelch(win_, 1, x + 1);
                mvwdelch(win_, 1, x - 1);
                wclrtoeol(win_);
                wrefresh(win_);

                if (input_buf.size() > 0)
                {
                    input_buf.erase(input_buf.size() - 1, 1);
                    x--;
                }
            }
            else
            {
                if (ch != '\n')
                {
                    input_buf.append(1, ch);
                }
                x++;
            }
        } while (ch != '\n');

        reset_input_window(win_);
        commandDecode(input_buf);
        refresh();
    }
}

void InputPanel::reset_input_window(WINDOW *win)
{
    werase(win);
    wclear(win);
    wrefresh(win);
}

void InputPanel::commandDecode(string buf)
{
    bool syntax_err = false;
    bool pb_selected = false;
    bool lm_selected = false;
    bool f_selected = false;
    LegModule *md_ptr_;

    vector<string> bufs;
    bufs = tokenizer(buf);

    if (bufs.size() >= 1)
    {
        if (bufs[0] == "P")
        {
            pb_selected = true;
        }
        else if (bufs[0] == "F")
        {
            f_selected = true;
        }
        else if (bufs[0] == "A")
        {
            lm_selected = true;
            md_ptr_ = modA_ptr_;
        }
        else if (bufs[0] == "B")
        {
            lm_selected = true;
            md_ptr_ = modB_ptr_;
        }
        else if (bufs[0] == "C")
        {
            lm_selected = true;
            md_ptr_ = modC_ptr_;
        }
        else if (bufs[0] == "D")
        {
            lm_selected = true;
            md_ptr_ = modD_ptr_;
        }
        else
        {
            syntax_err = true;
        }

        mvwprintw(win_, 2, 1, bufs[0].c_str());
        wrefresh(win_);
    }

    cons_mtx_.lock();
    main_mtx_->lock();

    if (bufs.size() == 3)
    {
        /* mvwprintw(win_, 1, 35, "buf= %d", bufs.size());
        mvwprintw(win_, 2, 35, "pb_ptr= %d", powerboard_state_); */

        mvwprintw(win_, 2, 3, bufs[1].c_str());
        mvwprintw(win_, 2, 5, bufs[2].c_str());
        wrefresh(win_);

        if (pb_selected)
        {
            if (bufs[1] == "D")
            {
                try
                {
                    powerboard_state_->at(0) = stoi(bufs[2]);
                }
                catch (exception &e)
                {
                    syntax_err = true;
                    mvwprintw(win_, 2, 1, "err");
                }
            }
            else if (bufs[1] == "S")
            {
                try
                {
                    powerboard_state_->at(1) = stoi(bufs[2]);
                }
                catch (exception &e)
                {
                    syntax_err = true;
                    mvwprintw(win_, 2, 1, "err");
                }
            }
            else if (bufs[1] == "P")
            {
                try
                {
                    powerboard_state_->at(2) = stoi(bufs[2]);
                }
                catch (exception &e)
                {
                    syntax_err = true;
                    mvwprintw(win_, 2, 1, "err");
                }
            }
            else
            {
                syntax_err = true;
            }
        }
        else if (f_selected)
        {
            if (bufs[1] == "M")
            {
                if (bufs[2] == "R")
                {
                    fsm_->switchMode(Mode::REST);
                }
                else if (bufs[2] == "M")
                {
                    fsm_->switchMode(Mode::MOTOR);
                }
                else if (bufs[2] == "S")
                {
                    fsm_->switchMode(Mode::SET_ZERO);
                }
                else if (bufs[2] == "H")
                {
                    fsm_->switchMode(Mode::HALL_CALIBRATE);
                }
                else
                {
                    syntax_err = true;
                }
            }
            else
            {
                syntax_err = true;
            }
        }
        else
        {
            syntax_err = true;
        }
    }
    else if (bufs.size() == 4)
    {
        mvwprintw(win_, 2, 3, bufs[1].c_str());
        mvwprintw(win_, 2, 5, bufs[2].c_str());
        mvwprintw(win_, 2, 7, bufs[3].c_str());
        wrefresh(win_);

        int mtr_idx = 0;
        if (bufs[1] == "F")
        {
            mtr_idx = 0;
        }
        else if (bufs[1] == "H")
        {
            mtr_idx = 1;
        }
        else
        {
            syntax_err = true;
        }

        if (!syntax_err)
        {
            if (bufs[2] == "C")
            {
                try
                {
                    md_ptr_->motors_list_[mtr_idx].CAN_ID_ = stoi(bufs[3]);
                }
                catch (exception &e)
                {
                    syntax_err = true;
                    mvwprintw(win_, 2, 1, "syntax error");
                }
            }
            else if (bufs[2] == "A")
            {
                try
                {
                    md_ptr_->txdata_buffer_[mtr_idx].position_ = stof(bufs[3]);
                }
                catch (exception &e)
                {
                    syntax_err = true;
                }
            }
            else if (bufs[2] == "T")
            {
                try
                {
                    md_ptr_->txdata_buffer_[mtr_idx].torque_ = stof(bufs[3]);
                }
                catch (exception &e)
                {
                    syntax_err = true;
                }
            }
            else if (bufs[2] == "P")
            {
                try
                {
                    md_ptr_->txdata_buffer_[mtr_idx].KP_ = stof(bufs[3]);
                }
                catch (exception &e)
                {
                    syntax_err = true;
                }
            }
            else if (bufs[2] == "I")
            {
                try
                {
                    md_ptr_->txdata_buffer_[mtr_idx].KI_ = stof(bufs[3]);
                }
                catch (exception &e)
                {
                    syntax_err = true;
                }
            }
            else if (bufs[2] == "D")
            {
                try
                {
                    md_ptr_->txdata_buffer_[mtr_idx].KD_ = stof(bufs[3]);
                }
                catch (exception &e)
                {
                    syntax_err = true;
                }
            }
            else
            {
                syntax_err = true;
            }
        }
    }
    else
    {
        syntax_err = true;
    }

    if (syntax_err)
    {
        mvwprintw(win_, 0, 1, "Syntax Error !");
    }
    else
    {
        mvwprintw(win_, 0, 1, "Command Send !");
    }

    cons_mtx_.unlock();
    main_mtx_->unlock();

    wrefresh(win_);
}

vector<string> InputPanel::tokenizer(string s)
{
    // A quick way to split strings separated via spaces.
    stringstream ss(s);
    string word;
    vector<string> bufs;
    while (ss >> word)
    {
        // cout << word << endl;
        bufs.push_back(word);
    }
    return bufs;
}

Panel::Panel(string title, string type, LegModule *lm_, int org_x, int org_y, int height, int width, bool box_on)
{
    org_x_ = org_x;
    org_y_ = org_y;
    height_ = height;
    width_ = width;
    // box_on_ = // box_on;
    type_ = type;
    title_ = title;

    md_ptr_ = lm_;

    win_ = newwin(height_, width_, org_y_, org_x_);
    refresh();

    // box(win_, 0, 0);

    string tag_(title.c_str(), title.c_str() + 3);
    title.erase(0, 3);

    // mvwprintw(win_, 0, (width_ / 2 - (title.size() + tag_.size()) / 2), tag_.c_str());
    wattron(win_, COLOR_PAIR(CYAN_PAIR));
    wattron(win_, A_BOLD);
    wattron(win_, A_STANDOUT);
    mvwprintw(win_, 0, (width_ / 2 - title.size() / 2 - 2), tag_.c_str());
    wattroff(win_, COLOR_PAIR(CYAN_PAIR));

    mvwprintw(win_, 0, (width_ / 2 - title.size() / 2 + 1), title.c_str());
    wattroff(win_, A_BOLD);
    wattroff(win_, A_STANDOUT);
    wrefresh(win_);
    // refresh();
}

void Panel::infoDisplay()
{
    int y_org = 2;

    // Motor_R
    mvwprintw(win_, 1, 1, "[F] Motor_R-----------------------------------------------");
    mvwprintw(win_, 2, 1, "[C] [CAN] ID: %9d", md_ptr_->motors_list_[0].CAN_ID_);
    mvwprintw(win_, 3, 1, "    [tx] TIMEDOUT: %4d", md_ptr_->CAN_tx_timedout_[0]);
    mvwprintw(win_, y_org + 2, 1, "[A] [tx] Pos:  %5.5f", md_ptr_->txdata_buffer_[0].position_);
    mvwprintw(win_, y_org + 3, 1, "[T] [tx] Trq:  %5.5f", md_ptr_->txdata_buffer_[0].torque_);
    mvwprintw(win_, y_org + 4, 1, "[P] [tx] KP:   %4.5f", md_ptr_->txdata_buffer_[0].KP_);
    mvwprintw(win_, y_org + 5, 1, "[I] [tx] KI:   %5.5f", md_ptr_->txdata_buffer_[0].KI_);
    mvwprintw(win_, y_org + 6, 1, "[D] [tx] KD:   %5.5f", md_ptr_->txdata_buffer_[0].KD_);
    // reply
    mvwprintw(win_, 3, 30, "[rx] TIMEDOUT: %4d", md_ptr_->CAN_rx_timedout_[0]);
    mvwprintw(win_, y_org + 2, 30, "[rx] Ver:   %7d", md_ptr_->rxdata_buffer_[0].version_);
    mvwprintw(win_, y_org + 3, 30, "[rx] Mode:  %7d", md_ptr_->rxdata_buffer_[0].mode_state_);
    mvwprintw(win_, y_org + 4, 30, "[rx] Pos:   %4.5f", md_ptr_->rxdata_buffer_[0].position_);
    mvwprintw(win_, y_org + 5, 30, "[rx] Vel:   %4.5f", md_ptr_->rxdata_buffer_[0].velocity_);
    mvwprintw(win_, y_org + 6, 30, "[rx] Trq:   %4.5f", md_ptr_->rxdata_buffer_[0].torque_);
    mvwprintw(win_, y_org + 7, 30, "[rx] Cal:   %7d", md_ptr_->rxdata_buffer_[0].calibrate_finish_);

    // Motor H
    mvwprintw(win_, 10, 1, "[H] Motor_L-----------------------------------------------");
    mvwprintw(win_, 11, 1, "[C] [CAN] ID: %9d", md_ptr_->motors_list_[1].CAN_ID_);
    mvwprintw(win_, 12, 1, "    [tx] TIMEDOUT: %4d", md_ptr_->CAN_tx_timedout_[1]);
    mvwprintw(win_, y_org + 11, 1, "[A] [tx] Pos:  %5.5f", md_ptr_->txdata_buffer_[1].position_);
    mvwprintw(win_, y_org + 12, 1, "[T] [tx] Trq:  %5.5f", md_ptr_->txdata_buffer_[1].torque_);
    mvwprintw(win_, y_org + 13, 1, "[P] [tx] KP:   %4.5f", md_ptr_->txdata_buffer_[1].KP_);
    mvwprintw(win_, y_org + 14, 1, "[I] [tx] KI:   %5.5f", md_ptr_->txdata_buffer_[1].KI_);
    mvwprintw(win_, y_org + 15, 1, "[D] [tx] KD:   %5.5f", md_ptr_->txdata_buffer_[1].KD_);
    // reply
    mvwprintw(win_, 12, 30, "[rx] TIMEDOUT: %4d", md_ptr_->CAN_rx_timedout_[1]);
    mvwprintw(win_, y_org + 11, 30, "[rx] Ver:   %7d", md_ptr_->rxdata_buffer_[1].version_);
    mvwprintw(win_, y_org + 12, 30, "[rx] Mode:  %7d", md_ptr_->rxdata_buffer_[1].mode_state_);
    mvwprintw(win_, y_org + 13, 30, "[rx] Pos:   %4.5f", md_ptr_->rxdata_buffer_[1].position_);
    mvwprintw(win_, y_org + 14, 30, "[rx] Vel:   %4.5f", md_ptr_->rxdata_buffer_[1].velocity_);
    mvwprintw(win_, y_org + 15, 30, "[rx] Trq:   %4.5f", md_ptr_->rxdata_buffer_[1].torque_);
    mvwprintw(win_, y_org + 16, 30, "[rx] Cal:   %7d", md_ptr_->rxdata_buffer_[1].calibrate_finish_);
    wrefresh(win_);
}

void Panel::infoDisplay(bool digital_switch, bool signal_switch, bool power_switch)
{
    mvwprintw(win_, 2, 1, "HARDWARE POWER SWITCH-----------------");
    mvwprintw(win_, 3, 1, "[D] Digital:   %4d", digital_switch);
    mvwprintw(win_, 4, 1, "[S] Signal:    %4d", signal_switch);
    mvwprintw(win_, 5, 1, "[P] Power:     %4d", power_switch);
    wrefresh(win_);
}

void Panel::infoDisplay(Behavior bhv, Mode fsm_mode)
{
    if (bhv == Behavior::TCP_SLAVE)
        mvwprintw(win_, 2, 1, "Behavior: TCP_SLAVE");
    else if (bhv == Behavior::SET_THETA)
        mvwprintw(win_, 2, 1, "Behavior: SET_THETA");
    else if (bhv == Behavior::CUSTOM_1)
        mvwprintw(win_, 2, 1, "Behavior: CUSTOM_1");
    else if (bhv == Behavior::CUSTOM_1)
        mvwprintw(win_, 2, 1, "Behavior: CUSTOM_2");
    else if (bhv == Behavior::CUSTOM_1)
        mvwprintw(win_, 2, 1, "Behavior: CUSTOM_3");

    if (fsm_mode == Mode::REST)
        mvwprintw(win_, 3, 1, "[M] FSM Mode:           REST");
    else if (fsm_mode == Mode::SET_ZERO)
        mvwprintw(win_, 3, 1, "[M] FSM Mode:       SET_ZERO");
    else if (fsm_mode == Mode::HALL_CALIBRATE)
        mvwprintw(win_, 3, 1, "[M] FSM Mode: HALL_CALIBRATE");
    else if (fsm_mode == Mode::MOTOR)
        mvwprintw(win_, 3, 1, "[M] FSM Mode:          MOTOR");

    mvwprintw(win_, 5, 1, "[R] REST  [S] SET_ZERO");
    mvwprintw(win_, 6, 1, "[M] MOTOR [H] HALL_CALIBRATE");

    wrefresh(win_);
}

void Panel::panelTitle()
{
    // box(win_, 0, 0);

    string tag_(title_.c_str(), title_.c_str() + 3);
    title_.erase(0, 3);

    // mvwprintw(win_, 0, (width_ / 2 - (title.size() + tag_.size()) / 2), tag_.c_str());
    wattron(win_, COLOR_PAIR(CYAN_PAIR));
    wattron(win_, A_BOLD);
    wattron(win_, A_STANDOUT);
    mvwprintw(win_, 0, (width_ / 2 - title_.size() / 2 - 2), tag_.c_str());
    wattroff(win_, COLOR_PAIR(CYAN_PAIR));

    mvwprintw(win_, 0, (width_ / 2 - title_.size() / 2 + 1), title_.c_str());
    wattroff(win_, A_BOLD);
    wattroff(win_, A_STANDOUT);

    wrefresh(win_);
}

void Panel::resetPanel()
{
    werase(win_);
    wclear(win_);
    wrefresh(win_);
}

/* int main()
{
    LegModule lm;
    Motor mt;
    mt.CAN_ID_ = 19;
    lm.motors_list_.push_back(mt);
    lm.motors_list_.push_back(mt);
    lm.CAN_tx_timedout_[0] = 0;
    lm.CAN_tx_timedout_[1] = 1;
    lm.CAN_rx_timedout_[0] = 0;
    lm.CAN_rx_timedout_[1] = 1;

    lm.motors_list_[0].CAN_ID_ = 19;
    lm.txdata_buffer_[0].position_ = 3.14156;
    lm.txdata_buffer_[0].torque_ = 3.4;
    lm.txdata_buffer_[0].KP_ = 50;
    lm.txdata_buffer_[0].KI_ = 0;
    lm.txdata_buffer_[0].KD_ = 1.50;

    lm.rxdata_buffer_[0].version_ = 1;
    lm.rxdata_buffer_[0].mode_state_ = 1;
    lm.rxdata_buffer_[0].position_ = 1.54;
    lm.rxdata_buffer_[0].velocity_ = 0.333;
    lm.rxdata_buffer_[0].torque_ = 1.332;
    lm.rxdata_buffer_[0].calibrate_finish_ = 1;

    lm.motors_list_[1].CAN_ID_ = 19;
    lm.txdata_buffer_[1].position_ = 3.14156;
    lm.txdata_buffer_[1].torque_ = 3.4;
    lm.txdata_buffer_[1].KP_ = 50;
    lm.txdata_buffer_[1].KI_ = 0;
    lm.txdata_buffer_[1].KD_ = 1.50;

    lm.rxdata_buffer_[1].version_ = 1;
    lm.rxdata_buffer_[1].mode_state_ = 1;
    lm.rxdata_buffer_[1].position_ = 1.54;
    lm.rxdata_buffer_[1].velocity_ = 0.333;
    lm.rxdata_buffer_[1].torque_ = 1.332;
    lm.rxdata_buffer_[1].calibrate_finish_ = 1;

    vector<LegModule> mods;
    mods.push_back(lm);
    mods.push_back(lm);
    mods.push_back(lm);
    mods.push_back(lm);
    Console console(mods);

    refresh();
    // getch();
    while (1)
    {
    }
    endwin();
    return 0;
}
 */