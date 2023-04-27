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
        /* if (!if_switch_mode_printed_)
        {
            // important_message("[FSM] REST MODE RUNNING ");
            // if_switch_mode_printed_ = true;
        } */
    }
    break;

    case Mode::SET_ZERO:
    {
        /* if (!if_switch_mode_printed_)
        {
            important_message(" [FSM] SET_ZERO MODE RUNNING ");
            if_switch_mode_printed_ = true;
        } */
    }
    break;

    case Mode::HALL_CALIBRATE:
    {
        /* if (!if_switch_mode_printed_)
        {
            important_message(" [FSM] HALL_CALIBRATE MODE RUNNING ");
            if_switch_mode_printed_ = true;
        } */

        // Left Link rotate 28 deg
        CAN_txdata txdata_pairAC[2];
        // Right Link rotate -28 deg
        CAN_txdata txdata_pairBD[2];

        txdata_pairAC[0].position_ = 0;
        txdata_pairAC[0].torque_ = 0;
        txdata_pairAC[0].KP_ = 10;
        txdata_pairAC[0].KI_ = 0;
        txdata_pairAC[0].KD_ = 1;

        txdata_pairAC[1].position_ = 45.5 * M_PI / 180.0;
        txdata_pairAC[1].torque_ = 0;
        txdata_pairAC[1].KP_ = 30;
        txdata_pairAC[1].KI_ = 0;
        txdata_pairAC[1].KD_ = 1.5;

        txdata_pairBD[0].position_ = -45.5 * M_PI / 180.0;
        txdata_pairBD[0].torque_ = 0;
        txdata_pairBD[0].KP_ = 30;
        txdata_pairBD[0].KI_ = 0;
        txdata_pairBD[0].KD_ = 1.5;

        txdata_pairBD[1].position_ = 0;
        txdata_pairBD[1].torque_ = 0;
        txdata_pairBD[1].KP_ = 10;
        txdata_pairBD[1].KI_ = 0;
        txdata_pairBD[1].KD_ = 1;

        int module_enabled = 0;
        int module_calibrated = 0;
        int module_setpose = 0;

        for (int i = 0; i < 4; i++)
        {
            if (modules_list_[i].enable_)
            {
                module_enabled++;
            }
        }

        /* while (1)
        {
            if (module_calibrated == module_enabled)
                break;
            else
                module_calibrated = 0;

            for (int i = 0; i < 4; i++)
            {
                if (modules_list_[i].enable_)
                {
                    modules_list_[i].io_.CAN_recieve_feedback(&modules_list_[i].rxdata_buffer_[0], &modules_list_[i].rxdata_buffer_[1]);
                }

                if (modules_list_[i].rxdata_buffer_[0].calibrate_finish_ == true && modules_list_[i].rxdata_buffer_[1].calibrate_finish_ == true)
                {
                    module_calibrated++;
                }
            }
        } */

        usleep(5 * 1000 * 1000);

        for (int i = 0; i < 4; i++)
        {
            if (modules_list_[i].enable_)
                modules_list_[i].io_.CAN_set_mode(Mode::MOTOR);
        }
        usleep(0.1 * 1000 * 1000);

        /* while (1)
        {
            if (module_setpose == module_enabled)
                break;
            else
                module_setpose = 0;

            for (int i = 0; i < 4; i++)
            {
                if (modules_list_[i].enable_)
                {
                    modules_list_[i].io_.CAN_recieve_feedback(&modules_list_[i].rxdata_buffer_[0], &modules_list_[i].rxdata_buffer_[1]);

                    if (i == 0 || i == 2)
                    {
                        modules_list_[i].io_.CAN_send_command(txdata_pairAC[0], txdata_pairAC[1]);
                    }
                    if (i == 1 || i == 3)
                    {
                        modules_list_[i].io_.CAN_send_command(txdata_pairBD[0], txdata_pairBD[1]);
                    }

                    if (fabs(modules_list_[i].rxdata_buffer_[0].position_ - modules_list_[i].txdata_buffer_[0].position_) < 0.05 &&
                        fabs(modules_list_[i].rxdata_buffer_[1].position_ - modules_list_[i].txdata_buffer_[1].position_) < 0.05)
                    {
                        module_setpose++;
                    }
                }
            }
            usleep(0.01 * 1000 * 1000);
        } */

        for (int i = 0; i < 4; i++)
        {
            if (modules_list_[i].enable_)
            {
                modules_list_[i].io_.CAN_recieve_feedback(&modules_list_[i].rxdata_buffer_[0], &modules_list_[i].rxdata_buffer_[1]);

                if (i == 0 || i == 2)
                {
                    modules_list_[i].io_.CAN_send_command(txdata_pairAC[0], txdata_pairAC[1]);
                }
                if (i == 1 || i == 3)
                {
                    modules_list_[i].io_.CAN_send_command(txdata_pairBD[0], txdata_pairBD[1]);
                }
            }
        }
        usleep(3 * 1000 * 1000);

        for (int i = 0; i < 4; i++)
        {
            if (modules_list_[i].enable_)
            {
                modules_list_[i].io_.CAN_set_mode(Mode::SET_ZERO);
            }
        }
        usleep(1 * 1000 * 1000);

        switchMode(Mode::REST);
    }
    break;

    case Mode::MOTOR:
    {
        /* if (!if_switch_mode_printed_)
        {
            important_message(" [FSM] MOTOR MODE RUNNING ");
            if_switch_mode_printed_ = true;
        } */
    }
    break;
    }
}

void ModeFsm::switchMode(Mode next_mode)
{

    int mode_switched_cnt = 0;
    int module_enabled = 0;

    for (int i = 0; i < 4; i++)
    {
        if (modules_list_[i].enable_)
        {
            module_enabled++;
        }
    }

    while (1)
    {
        if (mode_switched_cnt == module_enabled)
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

    prev_workingMode_ = workingMode_;
    workingMode_ = next_mode;
}
