#include <fsm.hpp>

ModeFsm::ModeFsm(std::vector<LegModule> *_modules, std::vector<bool> *_pb_state, double *pb_v)
{
    workingMode_ = Mode::REST;
    prev_workingMode_ = Mode::REST;

    modules_list_ = _modules;
    pb_state_ = _pb_state;
    powerboard_voltage = pb_v;

    hall_calibrated = false;
    hall_calibrate_status = 0;
    runFsm();
}

void ModeFsm::runFsm()
{
    switch (workingMode_)
    {
    case Mode::REST:
    {
        if (pb_state_->at(2) == true)
        {
            for (auto &mod : *modules_list_)
            {
                if (mod.enable_)
                {
                    mod.txdata_buffer_[0].position_ = 0;
                    mod.txdata_buffer_[0].torque_ = 0;
                    mod.txdata_buffer_[0].KP_ = 0;
                    mod.txdata_buffer_[0].KI_ = 0;
                    mod.txdata_buffer_[0].KD_ = 0;
                    mod.txdata_buffer_[1].position_ = 0;
                    mod.txdata_buffer_[1].torque_ = 0;
                    mod.txdata_buffer_[1].KP_ = 0;
                    mod.txdata_buffer_[1].KI_ = 0;
                    mod.txdata_buffer_[1].KD_ = 0;
                }
            }
        }
    }
    break;

    case Mode::SET_ZERO:
    {
        if (pb_state_->at(2) == true)
        {
            for (auto &mod : *modules_list_)
            {
                if (mod.enable_)
                {
                    mod.txdata_buffer_[0].position_ = 0;
                    mod.txdata_buffer_[0].torque_ = 0;
                    mod.txdata_buffer_[0].KP_ = 0;
                    mod.txdata_buffer_[0].KI_ = 0;
                    mod.txdata_buffer_[0].KD_ = 0;
                    mod.txdata_buffer_[1].position_ = 0;
                    mod.txdata_buffer_[1].torque_ = 0;
                    mod.txdata_buffer_[1].KP_ = 0;
                    mod.txdata_buffer_[1].KI_ = 0;
                    mod.txdata_buffer_[1].KD_ = 0;
                }
            }
        }
    }
    break;

    case Mode::HALL_CALIBRATE:
    {
        int power_off = 0;
        for (int i = 0; i < 12; i++)
        {
            double v = *(powerboard_voltage + i);
            if (v < 45)
            {
                power_off = 1;
            }
        }

        int module_enabled = 0;
        if (power_off || hall_calibrated == true){
            hall_calibrate_status = -1;
        }
        else{
            for (int i = 0; i < 4; i++)
            {
                if (modules_list_->at(i).enable_)
                {
                    modules_list_->at(i).txdata_buffer_[0].KP_ = 50;
                    modules_list_->at(i).txdata_buffer_[0].KI_ = 0;
                    modules_list_->at(i).txdata_buffer_[0].KD_ = 1.5;
                    modules_list_->at(i).txdata_buffer_[1].KP_ = 50;
                    modules_list_->at(i).txdata_buffer_[1].KI_ = 0;
                    modules_list_->at(i).txdata_buffer_[1].KD_ = 1.5;
                    module_enabled++;
                }
            }
        }


        switch (hall_calibrate_status)
        {
        case -1:
        {
            switchMode(Mode::REST);
        }
        break;

        case 0:
        {
            int cal_cnt = 0;
            for (int i = 0; i < 4; i++)
            {
                if (modules_list_->at(i).enable_)
                {
                    if (modules_list_->at(i).rxdata_buffer_[0].calibrate_finish_ == 2 && modules_list_->at(i).rxdata_buffer_[1].calibrate_finish_ == 2)
                        cal_cnt++;
                }
            }
            if (cal_cnt == module_enabled && measure_offset == 0)
                hall_calibrate_status++;
            else if (cal_cnt == module_enabled && measure_offset == 1)
                hall_calibrate_status = -1;
        }
        break;

        case 1:
        {
            for (int i = 0; i < 4; i++)
            {
                if (modules_list_->at(i).enable_)
                {
                    modules_list_->at(i).CAN_rx_timedout_[0] = false;
                    modules_list_->at(i).CAN_rx_timedout_[1] = false;
                    modules_list_->at(i).CAN_tx_timedout_[0] = false;
                    modules_list_->at(i).CAN_tx_timedout_[1] = false;

                    modules_list_->at(i).io_.motorR_bias = modules_list_->at(i).linkR_bias;
                    modules_list_->at(i).io_.motorL_bias = modules_list_->at(i).linkL_bias;

                    cal_command[i][0] = modules_list_->at(i).rxdata_buffer_[0].position_ - modules_list_->at(i).linkR_bias;
                    modules_list_->at(i).txdata_buffer_[0].position_ = modules_list_->at(i).rxdata_buffer_[0].position_ - modules_list_->at(i).linkR_bias;

                    if (theta_error(modules_list_->at(i).rxdata_buffer_[0].position_ - modules_list_->at(i).linkR_bias, 0) > 0)
                        cal_dir_[i][0] = 1;
                    else
                        cal_dir_[i][0] = -1;

                    cal_command[i][1] = modules_list_->at(i).rxdata_buffer_[1].position_ - modules_list_->at(i).linkL_bias;
                    modules_list_->at(i).txdata_buffer_[1].position_ = modules_list_->at(i).rxdata_buffer_[1].position_ - modules_list_->at(i).linkL_bias;
                    if (theta_error(modules_list_->at(i).rxdata_buffer_[1].position_ - modules_list_->at(i).linkL_bias, 0) > 0)
                        cal_dir_[i][1] = 1;
                    else
                        cal_dir_[i][1] = -1;
                }
            }
            hall_calibrate_status++;
        }
        break;

        case 2:
        {
            int finish_cnt = 0;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    double errj = 0;
                    errj = theta_error(cal_command[i][j], 0);
                    modules_list_->at(i).txdata_buffer_[j].position_ = 0;

                    if (fabs(errj) < cal_tol_)
                    {
                        modules_list_->at(i).txdata_buffer_[j].position_ = 0;
                        finish_cnt++;
                    }
                    else
                    {
                        cal_command[i][j] += cal_dir_[i][j] * cal_vel_ * dt_;

                        modules_list_->at(i).txdata_buffer_[j].position_ = cal_command[i][j];
                        modules_list_->at(i).txdata_buffer_[j].torque_ = 0;
                        modules_list_->at(i).txdata_buffer_[j].KP_ = 50;
                        modules_list_->at(i).txdata_buffer_[j].KI_ = 0;
                        modules_list_->at(i).txdata_buffer_[j].KD_ = 1.5;
                    }
                }
            }
            if (finish_cnt == 2 * module_enabled)
                hall_calibrate_status++;
        }
        break;

        case 3:
        {
            hall_calibrated = true;
            switchMode(Mode::MOTOR);
        }
        break;
        }
    }
    break;

    case Mode::MOTOR:
    {
    }
    break;
    }
}

bool ModeFsm::switchMode(Mode next_mode)
{
    int mode_switched_cnt = 0;
    int module_enabled = 0;
    bool success = false;

    for (int i = 0; i < 4; i++)
    {
        if (modules_list_->at(i).enable_)
        {
            module_enabled++;
        }
    }

    if (next_mode == Mode::HALL_CALIBRATE)
    {
        hall_calibrated = false;
        hall_calibrate_status = 0;
    }

    double time_elapsed = 0;

    while (1)
    {
        if (mode_switched_cnt == module_enabled)
        {
            success = true;
            break;
        }
        else if (time_elapsed > 1)
        {
            /* Timeout */
            success = false;
            break;
        }
        else
            mode_switched_cnt = 0;

        for (int i = 0; i < 4; i++)
        {
            if (modules_list_->at(i).enable_)
            {
                modules_list_->at(i).io_.CAN_set_mode(next_mode);

                modules_list_->at(i).io_.CAN_recieve_feedback(&modules_list_->at(i).rxdata_buffer_[0], &modules_list_->at(i).rxdata_buffer_[1]);

                if (modules_list_->at(i).rxdata_buffer_[0].mode_ == next_mode && modules_list_->at(i).rxdata_buffer_[1].mode_ == next_mode)
                {
                    mode_switched_cnt++;
                }
            }
        }

        time_elapsed += 0.01;
        usleep(1e4);
    }

    prev_workingMode_ = workingMode_;
    workingMode_ = next_mode;

    return success;
}
