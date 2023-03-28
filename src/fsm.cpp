#include <fsm.hpp>

ModeFsm::ModeFsm(std::vector<LegModule> &_modules)
{
    workingMode_ = Mode::REST;
    prev_workingMode_ = Mode::REST;

    modules_list_ = _modules;
    if_switch_mode_msg_sent_ = false;
    if_switch_mode_printed_ = false;
    runFsm();
}

void ModeFsm::runFsm()
{
    switch (workingMode_)
    {
    case Mode::REST:
    {
        if (!if_switch_mode_printed_)
        {
            important_message("[FSM] REST MODE RUNNING ");
            if_switch_mode_printed_ = true;
        }
    }
    break;

    case Mode::SET_ZERO:
    {
        if (!if_switch_mode_printed_)
        {
            important_message(" [FSM] SET_ZERO MODE RUNNING ");
            if_switch_mode_printed_ = true;
        }
    }
    break;

    case Mode::HALL_CALIBRATE:
    {
        if (!if_switch_mode_printed_)
        {
            important_message(" [FSM] HALL_CALIBRATE MODE RUNNING ");
            if_switch_mode_printed_ = true;
        }
    }
    break;

    case Mode::MOTOR:
    {
        if (!if_switch_mode_printed_)
        {
            important_message(" [FSM] MOTOR MODE RUNNING ");
            if_switch_mode_printed_ = true;
        }
    }
    break;
    }
}

void ModeFsm::switchMode(Mode next_mode)
{

    int mode_switched_cnt = 0;

    // for (int i = 0; i < 4; i++)
    // {
    //     if (modules_list_[i].enable_)
    //     {
    //         modules_list_[i].io_.CAN_set_mode(next_mode);
    //     }
    // }

    if (next_mode == Mode::MOTOR)
    {
        while (1)
        {
            if (mode_switched_cnt == 4)
                break;
            else
                mode_switched_cnt = 0;

            for (int i = 0; i < 4; i++)
            {
                if (modules_list_[i].enable_)
                {
                    modules_list_[i].io_.CAN_set_mode(next_mode);

                    modules_list_[i].io_.CAN_recieve_feedback(&modules_list_[i].rxdata_buffer_[0], &modules_list_[i].rxdata_buffer_[1]);

                    if (modules_list_[i].rxdata_buffer_[0].mode_ == next_mode && modules_list_[i].rxdata_buffer_[1].mode_ == next_mode)
                    {
                        mode_switched_cnt++;
                    }
                }
            }
        }
    }

    prev_workingMode_ = workingMode_;
    workingMode_ = next_mode;

    // usleep(1000 * 1000);

    // for (auto &mod : modules_list_)
    // {
    //     if (!if_switch_mode_msg_sent_)
    //     {
    //         mod.io_.CAN_set_mode(next_mode);
    //     }

    //     mod.io_.CAN_recieve_feedback(&mod.rxdata_buffer_[0], &mod.rxdata_buffer_[1]);

    //     if (mod.rxdata_buffer_[0].mode_ == next_mode && mod.rxdata_buffer_[1].mode_ == next_mode)
    //     {
    //         mode_switched_cnt++;
    //     }

    //     if (mode_switched_cnt == modules_list_.size())
    //     {
    //         prev_workingMode_ = workingMode_;
    //         workingMode_ = next_mode;
    //         if_switch_mode_msg_sent_ = false;
    //         if_switch_mode_printed_ = false;
    //         // important_message(" [FSM] Mode Switched Successfully ");
    //     }
    // }
}

void ModeFsm::waitFor(int iter)
{
}
