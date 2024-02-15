#include <fsm.hpp>

ModeFsm::ModeFsm(std::vector<LegModule>* _modules, std::vector<bool>* _pb_state, double* pb_v)
{
    workingMode_ = Mode::REST;
    prev_workingMode_ = Mode::REST;

    modules_list_ = _modules;
    pb_state_ = _pb_state;
    powerboard_voltage = pb_v;

    hall_calibrated = false;
    hall_calibrate_status = 0;
    impedance_status = 0;
}

void ModeFsm::runFsm(motor_msg::MotorStamped& motor_fb_msg, const motor_msg::MotorStamped& motor_cmd_msg,
                     force_msg::LegForceStamped& force_fb_msg, const force_msg::LegForceStamped& force_cmd_msg)
{
    switch (workingMode_)
    {
        case Mode::REST: {
            if (pb_state_->at(2) == true)
            {
                publishMsg(motor_fb_msg);
                for (auto& mod : *modules_list_)
                {
                    int index = 0;
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

        case Mode::SET_ZERO: {
            if (pb_state_->at(2) == true)
            {
                publishMsg(motor_fb_msg);
                for (auto& mod : *modules_list_)
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
                        // mod.force_tracker.trq_lpf_r.reset();
                        // mod.force_tracker.trq_lpf_l.reset();
                        // mod.force_tracker.vel_lpf_r.reset();
                        // mod.force_tracker.vel_lpf_l.reset();
                    }
                }
            }
        }
        break;

        case Mode::HALL_CALIBRATE: {
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

            switch (hall_calibrate_status)
            {
                case -1: {
                    switchMode(Mode::REST);
                }
                break;

                case 0: {
                    int cal_cnt = 0;
                    for (int i = 0; i < 4; i++)
                    {
                        if (modules_list_->at(i).enable_)
                        {
                            if (modules_list_->at(i).rxdata_buffer_[0].calibrate_finish_ == 2 &&
                                modules_list_->at(i).rxdata_buffer_[1].calibrate_finish_ == 2)
                                cal_cnt++;
                        }
                    }
                    if (cal_cnt == module_enabled && measure_offset == 0)
                        hall_calibrate_status++;
                    else if (cal_cnt == module_enabled && measure_offset == 1)
                        hall_calibrate_status = -1;
                }
                break;

                case 1: {
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

                            cal_command[i][0] =
                                modules_list_->at(i).rxdata_buffer_[0].position_ - modules_list_->at(i).linkR_bias;
                            modules_list_->at(i).txdata_buffer_[0].position_ =
                                modules_list_->at(i).rxdata_buffer_[0].position_ - modules_list_->at(i).linkR_bias;

                            if (theta_error(modules_list_->at(i).rxdata_buffer_[0].position_ -
                                                modules_list_->at(i).linkR_bias,
                                            0) > 0)
                                cal_dir_[i][0] = 1;
                            else
                                cal_dir_[i][0] = -1;

                            cal_command[i][1] =
                                modules_list_->at(i).rxdata_buffer_[1].position_ - modules_list_->at(i).linkL_bias;
                            modules_list_->at(i).txdata_buffer_[1].position_ =
                                modules_list_->at(i).rxdata_buffer_[1].position_ - modules_list_->at(i).linkL_bias;
                            if (theta_error(modules_list_->at(i).rxdata_buffer_[1].position_ -
                                                modules_list_->at(i).linkL_bias,
                                            0) > 0)
                                cal_dir_[i][1] = 1;
                            else
                                cal_dir_[i][1] = -1;
                        }
                    }
                    hall_calibrate_status++;
                }
                break;

                case 2: {
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

                case 3: {
                    hall_calibrated = true;
                    switchMode(Mode::MOTOR);
                }
                break;
            }
        }
        break;

        case Mode::MOTOR: {
            /* Pubish feedback data from Motors */
            publishMsg(motor_fb_msg);

            int index = 0;
            for (auto& mod : *modules_list_)
            {
                if (mod.enable_)
                {
                    /* Subscribe command from other nodes */
                    // initialize message
                    // update
                    if (*NO_CAN_TIMEDOUT_ERROR_ && *NO_SWITCH_TIMEDOUT_ERROR_ && motor_cmd_msg.motors().size() == 8)
                    {
                        if (cmd_type_ == Command_type::THETA_BETA)
                        {
                            Eigen::Vector2d tb_cmd;
                            // Full Robot experiment scenario
                            if (scenario_ == Scenario::ROBOT)
                            {
                                // Special Case for module A D should be inverted
                                if (index == 0 || index == 3)
                                {
                                    tb_cmd << motor_cmd_msg.legs(index).theta(), -1 * motor_cmd_msg.legs(index).beta();
                                }
                                else
                                {
                                    tb_cmd << motor_cmd_msg.legs(index).theta(), motor_cmd_msg.legs(index).beta();
                                }
                            }
                            // Single Module experiment scenario
                            else
                            {
                                tb_cmd << motor_cmd_msg.legs(index).theta(), motor_cmd_msg.legs(index).beta();
                            }
                            Eigen::Vector2d phi_cmd = tb2phi(tb_cmd);
                            mod.txdata_buffer_[0].position_ = phi_cmd[0];
                            mod.txdata_buffer_[1].position_ = phi_cmd[1];
                        }
                        else
                        {
                            // Command Type Phi_R Phi_L
                            mod.txdata_buffer_[0].position_ = motor_cmd_msg.motors(index * 2).angle();
                            mod.txdata_buffer_[1].position_ = motor_cmd_msg.motors(index * 2 + 1).angle();
                        }

                        mod.txdata_buffer_[0].torque_ = motor_cmd_msg.motors(index * 2).torque();
                        mod.txdata_buffer_[1].torque_ = motor_cmd_msg.motors(index * 2 + 1).torque();
                        mod.txdata_buffer_[0].KP_ = motor_cmd_msg.motors(index * 2).kp();
                        mod.txdata_buffer_[0].KI_ = motor_cmd_msg.motors(index * 2).ki();
                        mod.txdata_buffer_[0].KD_ = motor_cmd_msg.motors(index * 2).kd();
                        mod.txdata_buffer_[1].KP_ = motor_cmd_msg.motors(index * 2 + 1).kp();
                        mod.txdata_buffer_[1].KI_ = motor_cmd_msg.motors(index * 2 + 1).ki();
                        mod.txdata_buffer_[1].KD_ = motor_cmd_msg.motors(index * 2 + 1).kd();
                    }
                }
                index++;
            }
        }
        break;

        case Mode::IMPEDANCE: {
            /* Pubish feedback data from Motors */
            publishMsg(motor_fb_msg);

            switch (impedance_status)
            {
                case 0: {
                    for (auto& mod : *modules_list_)
                    {
                        if (mod.enable_)
                        {
                            Eigen::Vector2d phi_(mod.rxdata_buffer_[0].position_, mod.rxdata_buffer_[1].position_);
                            Eigen::Vector2d tb_ = phi2tb(phi_);
                            mod.force_tracker.initialize(tb_);
                        }
                    }
                    impedance_status++;
                }
                break;
                case 1: {
                    int idx = 0;
                    for (auto& mod : *modules_list_)
                    {
                        if (mod.enable_ && *NO_CAN_TIMEDOUT_ERROR_ && *NO_SWITCH_TIMEDOUT_ERROR_ &&
                            force_cmd_msg.force().size() == 4)
                        {
                            // X_d, F_d
                            double x_d = force_cmd_msg.force(idx).pose_x();
                            double y_d = force_cmd_msg.force(idx).pose_y();
                            double f_x = force_cmd_msg.force(idx).force_x();
                            double f_y = force_cmd_msg.force(idx).force_y();

                            if (idx == 0 || idx == 3)
                            {
                                // Special case for A, D module
                                x_d *= -1;
                                f_x *= -1;
                            }
                            Eigen::Vector2d X_d(x_d, y_d);
                            Eigen::Vector2d F_d(f_x, f_y);

                            // load param from command
                            mod.force_tracker.M_d(0, 0) = force_cmd_msg.impedance(idx).m_x();
                            mod.force_tracker.M_d(1, 1) = force_cmd_msg.impedance(idx).m_y();
                            mod.force_tracker.K_0(0, 0) = force_cmd_msg.impedance(idx).k0_x();
                            mod.force_tracker.K_0(1, 1) = force_cmd_msg.impedance(idx).k0_y();
                            mod.force_tracker.D_d(0, 0) = force_cmd_msg.impedance(idx).d_x();
                            mod.force_tracker.D_d(1, 1) = force_cmd_msg.impedance(idx).d_y();
                            mod.force_tracker.adaptive_kp[0] = force_cmd_msg.impedance(idx).adaptive_kp_x();
                            mod.force_tracker.adaptive_kp[1] = force_cmd_msg.impedance(idx).adaptive_kp_y();
                            mod.force_tracker.adaptive_ki[0] = force_cmd_msg.impedance(idx).adaptive_ki_x();
                            mod.force_tracker.adaptive_ki[1] = force_cmd_msg.impedance(idx).adaptive_ki_y();
                            mod.force_tracker.adaptive_kd[0] = force_cmd_msg.impedance(idx).adaptive_kd_x();
                            mod.force_tracker.adaptive_kd[1] = force_cmd_msg.impedance(idx).adaptive_kd_y();

                            double vel_filt_r = mod.force_tracker.vel_lpf_r.y_k;
                            double vel_filt_l = mod.force_tracker.vel_lpf_l.y_k;
                            double trq_filt_r = mod.force_tracker.trq_lpf_r.y_k;
                            double trq_filt_l = mod.force_tracker.trq_lpf_l.y_k;

                            Eigen::Vector2d phi_fb(mod.rxdata_buffer_[0].position_, mod.rxdata_buffer_[1].position_);
                            Eigen::Vector2d tb_fb = phi2tb(phi_fb);
                            Eigen::Vector2d trq_fb(trq_filt_r, trq_filt_l);
                            Eigen::Vector2d phi_vel(vel_filt_r, vel_filt_l);
                            // trq_fb = trq_fb * 2.2; // KT compensation
                            Eigen::Vector2d phi_cmd = mod.force_tracker.controlLoop(X_d, F_d, tb_fb, trq_fb, phi_vel);

                            mod.txdata_buffer_[0].position_ = phi_cmd[0];
                            mod.txdata_buffer_[1].position_ = phi_cmd[1];
                            mod.txdata_buffer_[0].torque_ = 0;
                            mod.txdata_buffer_[1].torque_ = 0;
                            mod.txdata_buffer_[0].KP_ = 90;
                            mod.txdata_buffer_[0].KI_ = 0;
                            mod.txdata_buffer_[0].KD_ = 1.75;
                            mod.txdata_buffer_[1].KP_ = 90;
                            mod.txdata_buffer_[1].KI_ = 0;
                            mod.txdata_buffer_[1].KD_ = 1.75;

                            Eigen::Vector2d F_fb = mod.force_tracker.F_fb_q[0];
                            // Publish Feedback force data
                            force_msg::LegForce f;
                            Eigen::Vector2d xy_fb = fk(tb_fb);
                            f.set_force_x(F_fb[0]);
                            f.set_force_y(F_fb[1]);
                            f.set_pose_x(xy_fb[0]);
                            f.set_pose_x(xy_fb[1]);
                            force_msg::Impedance imp;
                            imp.set_m_x(mod.force_tracker.M_d(0,0));
                            imp.set_m_y(mod.force_tracker.M_d(1,1));
                            imp.set_k0_x(mod.force_tracker.K_ft(0,0));
                            imp.set_k0_y(mod.force_tracker.K_ft(1,1));
                            imp.set_d_x(mod.force_tracker.D_d(0,0));
                            imp.set_d_y(mod.force_tracker.D_d(1,1));
                            imp.set_adaptive_kp_x(mod.force_tracker.adaptive_kp[0]);
                            imp.set_adaptive_kp_y(mod.force_tracker.adaptive_kp[1]);
                            imp.set_adaptive_ki_x(mod.force_tracker.adaptive_ki[0]);
                            imp.set_adaptive_ki_y(mod.force_tracker.adaptive_ki[1]);
                            imp.set_adaptive_kd_x(mod.force_tracker.adaptive_kd[0]);
                            imp.set_adaptive_kd_y(mod.force_tracker.adaptive_kd[1]);
                            force_fb_msg.add_force()->CopyFrom(f);
                            force_fb_msg.add_impedance()->CopyFrom(imp);
                        }
                        else
                        {
                            force_msg::LegForce f;
                            f.set_force_x(0);
                            f.set_force_y(0);
                            force_fb_msg.add_force()->CopyFrom(f);
                        }
                        idx++;
                    }
                }
                break;
                case 2: {
                }
                break;
            }
        }
        break;
    }
}

bool ModeFsm::switchMode(Mode next_mode)
{
    int mode_switched_cnt = 0;
    int module_enabled = 0;
    bool success = false;
    Mode next_mode_switch = next_mode;

    for (int i = 0; i < 4; i++)
    {
        if (modules_list_->at(i).enable_)
        {
            module_enabled++;
        }
    }

    if (next_mode == Mode::HALL_CALIBRATE)
    {
        // Skip hall calibration if the modules have been calibrated
        if (hall_calibrated)
            next_mode_switch = workingMode_;
    }
    else if (next_mode == Mode::IMPEDANCE)
    {
        // For impedance mode, motors' state are position control mode
        next_mode_switch = Mode::MOTOR;
        impedance_status = 0;
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
                modules_list_->at(i).io_.CAN_set_mode(next_mode_switch);

                modules_list_->at(i).io_.CAN_recieve_feedback(&modules_list_->at(i).rxdata_buffer_[0],
                                                              &modules_list_->at(i).rxdata_buffer_[1]);

                if (modules_list_->at(i).rxdata_buffer_[0].mode_ == next_mode_switch &&
                    modules_list_->at(i).rxdata_buffer_[1].mode_ == next_mode_switch)
                {
                    mode_switched_cnt++;
                }
            }
        }

        time_elapsed += 0.01;
        usleep(1e4);
    }

    if (next_mode == Mode::IMPEDANCE)
        next_mode_switch = Mode::IMPEDANCE;

    prev_workingMode_ = workingMode_;
    workingMode_ = next_mode_switch;

    return success;
}

void ModeFsm::publishMsg(motor_msg::MotorStamped& motor_fb_msg)
{
    int index = 0;
    for (auto& mod : *modules_list_)
    {
        if (mod.enable_)
        {
            /* Pubish feedback data from Motors */
            motor_msg::Motor motor_r;
            motor_msg::Motor motor_l;
            motor_msg::LegAngle leg;
            motor_r.set_angle(mod.rxdata_buffer_[0].position_);  // phi R
            motor_l.set_angle(mod.rxdata_buffer_[1].position_);  // phi L

            Eigen::Vector2d phi_(mod.rxdata_buffer_[0].position_, mod.rxdata_buffer_[1].position_);
            Eigen::Vector2d tb_ = phi2tb(phi_);

            if (scenario_ == Scenario::SINGLE_MODULE)
            {
                leg.set_theta(tb_[0]);  // theta
                leg.set_beta(tb_[1]);   // beta
            }
            else
            {
                /* Special Case for Module A and D in Robot Scenario [ Module's beta frame should *-1 ]*/
                leg.set_theta(tb_[0]);  // theta
                if (index == 0 || index == 3)
                    leg.set_beta(-tb_[1]);  // beta
                else
                    leg.set_beta(tb_[1]);  // beta
            }
            double vel_filt_r = mod.force_tracker.vel_lpf_r.update(mod.rxdata_buffer_[0].velocity_);
            double vel_filt_l = mod.force_tracker.vel_lpf_l.update(mod.rxdata_buffer_[1].velocity_);
            double trq_filt_r = mod.force_tracker.trq_lpf_r.update(mod.rxdata_buffer_[0].torque_ * 2.2);
            double trq_filt_l = mod.force_tracker.trq_lpf_l.update(mod.rxdata_buffer_[1].torque_ * 2.2);
            motor_r.set_twist(vel_filt_r);   // velocity R
            motor_l.set_twist(vel_filt_l);   // velocity L
            motor_r.set_torque(trq_filt_r);  // torque R
            motor_l.set_torque(trq_filt_l);  // torque L
            // motor_r.set_twist(mod.rxdata_buffer_[0].velocity_); // velocity R
            // motor_l.set_twist(mod.rxdata_buffer_[1].velocity_); // velocity L
            // motor_r.set_torque(mod.rxdata_buffer_[0].torque_);  // torque R
            // motor_l.set_torque(mod.rxdata_buffer_[1].torque_);  // torque L

            // term << "raw_vel: " << mod.rxdata_buffer_[0].velocity_ << std::endl;
            // term << "vel_filt_r: " << vel_filt_r << std::endl;

            motor_fb_msg.add_motors()->CopyFrom(motor_r);
            motor_fb_msg.add_motors()->CopyFrom(motor_l);
            motor_fb_msg.add_legs()->CopyFrom(leg);
        }
        else
        {
            /* Pubish feedback data from Motors */
            motor_msg::Motor motor_r;
            motor_msg::Motor motor_l;
            motor_msg::LegAngle leg;
            motor_r.set_angle(0);   // phi R
            motor_l.set_angle(0);   // phi L
            leg.set_theta(0);       // theta
            leg.set_beta(0);        // beta
            motor_r.set_twist(0);   // velocity R
            motor_l.set_twist(0);   // velocity L
            motor_r.set_torque(0);  // torque R
            motor_l.set_torque(0);  // torque L
            motor_fb_msg.add_motors()->CopyFrom(motor_r);
            motor_fb_msg.add_motors()->CopyFrom(motor_l);
            motor_fb_msg.add_legs()->CopyFrom(leg);
        }
        index++;
    }
}
