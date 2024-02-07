#include <fpga_server.hpp>

/* TCP node connection setup*/
volatile int motor_message_updated = 0;
volatile int force_message_updated = 0;
volatile int fpga_message_updated = 0;
volatile bool vicon_toggle = true;

std::ofstream term;

std::mutex mutex_;
motor_msg::MotorStamped motor_cmd_data;
force_msg::LegForceStamped force_cmd_data;

void motor_data_cb(motor_msg::MotorStamped msg)
{
    mutex_.lock();
    motor_message_updated = 1;
    motor_cmd_data = msg;
    mutex_.unlock();
}

void force_data_cb(force_msg::LegForceStamped msg)
{
    mutex_.lock();
    force_message_updated = 1;
    force_cmd_data = msg;
    mutex_.unlock();
}

power_msg::PowerBoardStamped power_command_request;
power_msg::PowerBoardStamped power_dashboard_reply;

void power_command_function(power_msg::PowerBoardStamped request)
{
    mutex_.lock();
    fpga_message_updated = 1;
    power_command_request = request;
    mutex_.unlock();
}

void cb(power_msg::PowerBoardStamped request, power_msg::PowerBoardStamped &reply)
{
    power_command_function(request);
    mutex_.lock();
    reply = power_dashboard_reply;
    mutex_.unlock();
}

Corgi::Corgi()
{
    stop_ = false;

    /* default value of interrupt*/
    main_irq_period_us_ = 500;
    can_irq_period_us_ = 800;

    seq = 0;

    /* initialize powerboard state */
    digital_switch_ = false;
    signal_switch_ = false;
    power_switch_ = false;
    vicon_trigger_ = false;

    NO_CAN_TIMEDOUT_ERROR_ = true;
    NO_SWITCH_TIMEDOUT_ERROR_ = true;
    HALL_CALIBRATED_ = false;

    max_timeout_cnt_ = 100;

    powerboard_state_.push_back(digital_switch_);
    powerboard_state_.push_back(signal_switch_);
    powerboard_state_.push_back(power_switch_);

    ModeFsm fsm(&modules_list_, &powerboard_state_, fpga_.powerboard_V_list_);
    fsm_ = fsm;
    fsm_.NO_CAN_TIMEDOUT_ERROR_ = &NO_CAN_TIMEDOUT_ERROR_;
    fsm_.NO_SWITCH_TIMEDOUT_ERROR_ = &NO_SWITCH_TIMEDOUT_ERROR_;

    load_config_();

    console_.init(&fpga_, &modules_list_, &powerboard_state_, &fsm_, &main_mtx_);

    fpga_.setIrqPeriod(main_irq_period_us_, can_irq_period_us_);

    MSG_Stream.open("/home/admin/fpga_driver/log/log.txt");
}

void Corgi::load_config_()
{
    yaml_node_ = YAML::LoadFile(CONFIG_PATH);

    log_data = yaml_node_["log_data"].as<int>();
    log_path = yaml_node_["log_path"].as<std::string>();
    logger_init();

    fsm_.dt_ = yaml_node_["MainLoop_period_us"].as<int>() * 0.000001;
    fsm_.measure_offset = yaml_node_["Measure_offset"].as<int>();
    fsm_.cal_vel_ = yaml_node_["Hall_calibration_vel"].as<double>();
    fsm_.cal_tol_ = yaml_node_["Hall_calibration_tol"].as<double>();

    if (yaml_node_["Scenario"].as<std::string>().compare("SingleModule") == 0)
        fsm_.scenario_ = Scenario::SINGLE_MODULE;
    else
        fsm_.scenario_ = Scenario::ROBOT;

    if (yaml_node_["Command_type"].as<std::string>().compare("phiR_phiL") == 0)
        fsm_.cmd_type_ = Command_type::PHI_RL;
    else
        fsm_.cmd_type_ = Command_type::THETA_BETA;

    main_irq_period_us_ = yaml_node_["MainLoop_period_us"].as<int>();
    can_irq_period_us_ = yaml_node_["CANLoop_period_us"].as<int>();

    /* initialize leg modules */
    modules_num_ = yaml_node_["Number_of_modules"].as<int>();

    for (int i = 0; i < modules_num_; i++)
    {
        std::string label = yaml_node_["Modules_list"][i].as<std::string>();
        LegModule module(label, yaml_node_, fpga_.status_, fpga_.session_);
        modules_list_.push_back(module);
    }

    YAML::Node Factors_node_ = yaml_node_["Powerboard_Scaling_Factor"];
    int idx_ = 0;

    std::cout << "PowerBoard Scaling Factor" << std::endl;
    for (auto f : Factors_node_)
    {
        fpga_.powerboard_Ifactor[idx_] = f["Current_Factor"].as<double>();
        fpga_.powerboard_Vfactor[idx_] = f["Voltage_Factor"].as<double>();
        std::cout << "Index " << idx_ << " Current Factor: " << fpga_.powerboard_Ifactor[idx_] << ", Voltage Factor: " << fpga_.powerboard_Vfactor[idx_] << std::endl;
        idx_++;
    }
}

void Corgi::interruptHandler(core::ServiceServer<power_msg::PowerBoardStamped, power_msg::PowerBoardStamped> &power_srv,
                             core::Subscriber<motor_msg::MotorStamped> &cmd_sub_, core::Publisher<motor_msg::MotorStamped> &state_pub_,
                             core::Subscriber<force_msg::LegForceStamped> &force_sub, core::Publisher<force_msg::LegForceStamped> &force_pub)
{
    while (NiFpga_IsNotError(fpga_.status_) && !stop_ && !sys_stop)
    {
        uint32_t irqsAsserted;
        uint32_t irqTimeout = 10; // millisecond
        NiFpga_Bool TimedOut = 0;

        // Wait on IRQ to ensure FPGA is ready
        NiFpga_MergeStatus(&fpga_.status_,
                           NiFpga_WaitOnIrqs(fpga_.session_, fpga_.irqContext_, NiFpga_Irq_0 | NiFpga_Irq_1, irqTimeout,
                                             &irqsAsserted, &TimedOut));

        if (NiFpga_IsError(fpga_.status_))
        {
            std::cout << red << "[FPGA Server] Error! Exiting program. LabVIEW error code: " << fpga_.status_ << reset
                      << std::endl;
        }

        uint32_t irq0_cnt;
        uint32_t irq1_cnt;

        NiFpga_MergeStatus(
            &fpga_.status_,
            NiFpga_ReadU32(fpga_.session_, NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorI32_IRQ0_cnt, &irq0_cnt));

        NiFpga_MergeStatus(
            &fpga_.status_,
            NiFpga_ReadU32(fpga_.session_, NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorI32_IRQ0_cnt, &irq1_cnt));

        if (TimedOut)
        {
            std::cout << red << "IRQ timedout"
                      << ", IRQ_0 cnt: " << irq0_cnt << ", IRQ_1 cnt: " << irq1_cnt << reset << std::endl;
        }

        /* if an IRQ was asserted */
        if (NiFpga_IsNotError(fpga_.status_) && !TimedOut)
        {
            if (irqsAsserted & NiFpga_Irq_0)
            {
                /* TODO: do something if IRQ0 */
                mainLoop_(power_srv, cmd_sub_, state_pub_, force_sub, force_pub);

                // Acknowledge IRQ to begin DMA acquisition
                NiFpga_MergeStatus(&fpga_.status_, NiFpga_AcknowledgeIrqs(fpga_.session_, irqsAsserted));
            }
            if (irqsAsserted & NiFpga_Irq_1)
            {
                /* TODO: do something if IRQ1 */
                /* Handling CAN-BUS communication */
                canLoop_();

                // Acknowledge IRQ to begin DMA acquisition
                NiFpga_MergeStatus(&fpga_.status_, NiFpga_AcknowledgeIrqs(fpga_.session_, irqsAsserted));
            }
        }
        usleep(10);
    }
}

void Corgi::mainLoop_(core::ServiceServer<power_msg::PowerBoardStamped, power_msg::PowerBoardStamped> &power_srv,
                      core::Subscriber<motor_msg::MotorStamped> &cmd_sub_, core::Publisher<motor_msg::MotorStamped> &state_pub_,
                      core::Subscriber<force_msg::LegForceStamped> &force_sub, core::Publisher<force_msg::LegForceStamped> &force_pub)
{
    fpga_.write_powerboard_(&powerboard_state_);
    fpga_.read_powerboard_data_();

    core::spinOnce();
    mutex_.lock();

    motor_msg::MotorStamped motor_fb_msg;
    force_msg::LegForceStamped force_fb_msg;
    fsm_.runFsm(motor_fb_msg, motor_cmd_data, force_fb_msg, force_cmd_data);
    motor_message_updated = 0;
    force_message_updated = 0;

    HALL_CALIBRATED_ = fsm_.hall_calibrated;

    mutex_.unlock();

    // Communication with Node Architecture
    powerboardPack();

    // Read Command
    mutex_.lock();
    if ((*power_command_request.mutable_digital())["clean_error"] == true)
    {
        NO_SWITCH_TIMEDOUT_ERROR_ = true;
        NO_CAN_TIMEDOUT_ERROR_ = true;
        HALL_CALIBRATED_ = false;
        timeout_cnt_ = 0;
    }

    if (NO_SWITCH_TIMEDOUT_ERROR_)
    {
        if (fpga_message_updated)
        {
            powerboard_state_.at(0) = (*power_command_request.mutable_digital())["digital"];
            powerboard_state_.at(1) = (*power_command_request.mutable_digital())["signal"];
            powerboard_state_.at(2) = (*power_command_request.mutable_digital())["power"];
            fpga_.write_vicon_trigger((*power_command_request.mutable_digital())["vicon_trigger"]);
            vicon_trigger_ = (*power_command_request.mutable_digital())["vicon_trigger"];
            fpga_.write_orin_trigger((*power_command_request.mutable_digital())["orin_trigger"]);

            if (power_command_request.mode() == _REST_MODE && fsm_.workingMode_ != Mode::REST)
            {
                fsm_.switchMode(Mode::REST);
            }
            else if (power_command_request.mode() == _MOTOR_MODE && fsm_.workingMode_ != Mode::MOTOR)
            {
                fsm_.switchMode(Mode::MOTOR);
            }
            else if (power_command_request.mode() == _HALL_CALIBRATE && fsm_.workingMode_ != Mode::HALL_CALIBRATE)
            {
                fsm_.switchMode(Mode::HALL_CALIBRATE);
            }
            else if (power_command_request.mode() == _SET_ZERO && fsm_.workingMode_ != Mode::SET_ZERO)
            {
                fsm_.switchMode(Mode::SET_ZERO);
            }
            else if (power_command_request.mode() == _IMPEDANCE && fsm_.workingMode_ != Mode::IMPEDANCE)
            {
                fsm_.switchMode(Mode::IMPEDANCE);
            }

            /*if (fpga_common_control_data.mode == _REST_MODE)
                NO_SWITCH_TIMEDOUT_ERROR_ = NO_SWITCH_TIMEDOUT_ERROR_ && fsm_.switchMode(Mode::REST);
            else if (fpga_common_control_data.mode == _MOTOR_MODE)
                NO_SWITCH_TIMEDOUT_ERROR_ = NO_SWITCH_TIMEDOUT_ERROR_ && fsm_.switchMode(Mode::MOTOR);
            else if (fpga_common_control_data.mode == _HALL_CALIBRATE && !HALL_CALIBRATED_)
            {
                NO_SWITCH_TIMEDOUT_ERROR_ = NO_SWITCH_TIMEDOUT_ERROR_ && fsm_.switchMode(Mode::HALL_CALIBRATE);
            }
            else if (fpga_common_control_data.mode == _SET_ZERO)
                NO_SWITCH_TIMEDOUT_ERROR_ = NO_SWITCH_TIMEDOUT_ERROR_ && fsm_.switchMode(Mode::SET_ZERO);*/

            fpga_message_updated = 0;
        }
    }

    mutex_.unlock();
    state_pub_.publish(motor_fb_msg);
    force_pub.publish(force_fb_msg);

    logger(seq);
    seq++;
}

void Corgi::canLoop_()
{
    for (int i = 0; i < 4; i++)
    {
        if (modules_list_[i].enable_ && powerboard_state_.at(2) == true)
        {
            modules_list_[i].io_.CAN_recieve_feedback(&modules_list_[i].rxdata_buffer_[0], &modules_list_[i].rxdata_buffer_[1]);
            modules_list_[i].CAN_timeoutCheck();

            if (modules_list_[i].CAN_module_timedout)
            {
                timeout_cnt_++;
            }
            else
            {
                timeout_cnt_ = 0;
            }

            if (timeout_cnt_ < max_timeout_cnt_)
            {
                modules_list_[i].io_.CAN_send_command(modules_list_[i].txdata_buffer_[0], modules_list_[i].txdata_buffer_[1]);
                NO_CAN_TIMEDOUT_ERROR_ = true;
            }
            else
            {
                NO_CAN_TIMEDOUT_ERROR_ = false;
            }
        }
    }
}

void Corgi::powerboardPack()
{
    mutex_.lock();
    gettimeofday(&t_stamp, NULL);
    auto power_digital_dashboard = *power_dashboard_reply.mutable_digital();

    power_dashboard_reply.mutable_header()->set_seq(seq);
    power_dashboard_reply.mutable_header()->mutable_stamp()->set_sec(t_stamp.tv_sec);
    power_dashboard_reply.mutable_header()->mutable_stamp()->set_usec(t_stamp.tv_usec);

    power_digital_dashboard["digital"] = powerboard_state_.at(0);
    power_digital_dashboard["signal"] = powerboard_state_.at(1);
    power_digital_dashboard["power"] = powerboard_state_.at(2);
    power_digital_dashboard["hall_calibrate_enable"] = fsm_.hall_calibrated;
    power_digital_dashboard["SWITCH_TIMEDOUT"] = !NO_SWITCH_TIMEDOUT_ERROR_;
    power_digital_dashboard["CAN_TIMEDOUT"] = !NO_CAN_TIMEDOUT_ERROR_;

    if (fsm_.workingMode_ == Mode::REST)
        power_dashboard_reply.set_mode(power_msg::REST_MODE);
    else if (fsm_.workingMode_ == Mode::HALL_CALIBRATE)
        power_dashboard_reply.set_mode(power_msg::HALL_CALIBRATE);
    else if (fsm_.workingMode_ == Mode::MOTOR)
        power_dashboard_reply.set_mode(power_msg::MOTOR_MODE);
    else if (fsm_.workingMode_ == Mode::SET_ZERO)
        power_dashboard_reply.set_mode(power_msg::SET_ZERO);

    auto power_analog_dashboard = *power_dashboard_reply.mutable_analog();
    power_analog_dashboard["PB_1_Battery[0]"] = fpga_.powerboard_V_list_[0];
    power_analog_dashboard["PB_1_Battery[1]"] = fpga_.powerboard_I_list_[0];

    power_analog_dashboard["PB_2_CPU[0]"] = fpga_.powerboard_V_list_[1];
    power_analog_dashboard["PB_2_CPU[1]"] = fpga_.powerboard_I_list_[1];

    power_analog_dashboard["PB_3_SPARE1[0]"] = fpga_.powerboard_V_list_[2];
    power_analog_dashboard["PB_3_SPARE1[1]"] = fpga_.powerboard_I_list_[2];
    power_analog_dashboard["PB_4_SPARE2[0]"] = fpga_.powerboard_V_list_[3];
    power_analog_dashboard["PB_4_SPARE2[1]"] = fpga_.powerboard_I_list_[3];

    power_analog_dashboard["PB_5_M1_A_LF_L[0]"] = fpga_.powerboard_V_list_[4];
    power_analog_dashboard["PB_5_M1_A_LF_L[1]"] = fpga_.powerboard_I_list_[4];

    power_analog_dashboard["PB_6_M2_A_LF_R[0]"] = fpga_.powerboard_V_list_[5];
    power_analog_dashboard["PB_6_M2_A_LF_R[1]"] = fpga_.powerboard_I_list_[5];

    power_analog_dashboard["PB_7_M3_B_RF_L[0]"] = fpga_.powerboard_V_list_[6];
    power_analog_dashboard["PB_7_M3_B_RF_L[1]"] = fpga_.powerboard_I_list_[6];

    power_analog_dashboard["PB_8_M4_B_RF_R[0]"] = fpga_.powerboard_V_list_[7];
    power_analog_dashboard["PB_8_M4_B_RF_R[1]"] = fpga_.powerboard_I_list_[7];

    power_analog_dashboard["PB_9_M5_D_LH_R[0]"] = fpga_.powerboard_V_list_[8];
    power_analog_dashboard["PB_9_M5_D_LH_R[1]"] = fpga_.powerboard_I_list_[8];

    power_analog_dashboard["PB_10_M6_D_LH_L[0]"] = fpga_.powerboard_V_list_[9];
    power_analog_dashboard["PB_10_M6_D_LH_L[1]"] = fpga_.powerboard_I_list_[9];

    power_analog_dashboard["PB_11_M7_C_RH_R[0]"] = fpga_.powerboard_V_list_[10];
    power_analog_dashboard["PB_11_M7_C_RH_R[1]"] = fpga_.powerboard_I_list_[10];

    power_analog_dashboard["PB_12_M8_C_RH_L[0]"] = fpga_.powerboard_V_list_[11];
    power_analog_dashboard["PB_12_M8_C_RH_L[1]"] = fpga_.powerboard_I_list_[11];

    mutex_.unlock();
}

void Corgi::logger_init()
{
    if (log_data)
    {
        log_stream.open(log_path);
        log_stream << "seq,vicon_trigger,";
        log_stream << "AR_cmd_pos,AR_cmd_torq,AR_cmd_kp,AR_cmd_kd,AR_rpy_pos,AR_rpy_torq,";
        log_stream << "AL_cmd_pos,AL_cmd_torq,AL_cmd_kp,AL_cmd_kd,AL_rpy_pos,AL_rpy_torq,";
        log_stream << "BR_cmd_pos,BR_cmd_torq,BR_cmd_kp,BR_cmd_kd,BR_rpy_pos,BR_rpy_torq,";
        log_stream << "BL_cmd_pos,BL_cmd_torq,BL_cmd_kp,BL_cmd_kd,BL_rpy_pos,BL_rpy_torq,";
        log_stream << "CR_cmd_pos,CR_cmd_torq,CR_cmd_kp,CR_cmd_kd,CR_rpy_pos,CR_rpy_torq,";
        log_stream << "CL_cmd_pos,CL_cmd_torq,CL_cmd_kp,CL_cmd_kd,CL_rpy_pos,CL_rpy_torq,";
        log_stream << "DR_cmd_pos,DR_cmd_torq,DR_cmd_kp,DR_cmd_kd,DR_rpy_pos,DR_rpy_torq,";
        log_stream << "DL_cmd_pos,DL_cmd_torq,DL_cmd_kp,DL_cmd_kd,DL_rpy_pos,DL_rpy_torq,";
        log_stream << "Powerboard_0_V,Powerboard_0_I,";
        log_stream << "Powerboard_1_V,Powerboard_1_I,";
        log_stream << "Powerboard_2_V,Powerboard_2_I,";
        log_stream << "Powerboard_3_V,Powerboard_3_I,";
        log_stream << "Powerboard_4_V,Powerboard_4_I,";
        log_stream << "Powerboard_5_V,Powerboard_5_I,";
        log_stream << "Powerboard_6_V,Powerboard_6_I,";
        log_stream << "Powerboard_7_V,Powerboard_7_I,";
        log_stream << "Powerboard_8_V,Powerboard_8_I,";
        log_stream << "Powerboard_9_V,Powerboard_9_I,";
        log_stream << "Powerboard_10_V,Powerboard_10_I,";
        log_stream << "Powerboard_11_V,Powerboard_11_I,";
    }
}

void Corgi::logger(int seq)
{
    if (log_data)
    {
        // log data
        log_stream << seq << ",";
        // log_stream << (*power_command_request.mutable_digital())["vicon_trigger"] << ",";
        log_stream << vicon_trigger_ << ",";
        for (int i = 0; i < 4; i++)
        {
            log_stream << modules_list_.at(i).txdata_buffer_[0].position_ << ",";
            log_stream << modules_list_.at(i).txdata_buffer_[0].torque_ << ",";
            log_stream << modules_list_.at(i).txdata_buffer_[0].KP_ << ",";
            log_stream << modules_list_.at(i).txdata_buffer_[0].KD_ << ",";
            log_stream << modules_list_.at(i).rxdata_buffer_[0].position_ << ",";
            log_stream << modules_list_.at(i).rxdata_buffer_[0].torque_ << ",";

            log_stream << modules_list_.at(i).txdata_buffer_[1].position_ << ",";
            log_stream << modules_list_.at(i).txdata_buffer_[1].torque_ << ",";
            log_stream << modules_list_.at(i).txdata_buffer_[1].KP_ << ",";
            log_stream << modules_list_.at(i).txdata_buffer_[1].KD_ << ",";
            log_stream << modules_list_.at(i).rxdata_buffer_[1].position_ << ",";
            log_stream << modules_list_.at(i).rxdata_buffer_[1].torque_ << ",";
        }
        log_stream << fpga_.powerboard_V_list_[0] << "," << fpga_.powerboard_I_list_[0] << ",";
        log_stream << fpga_.powerboard_V_list_[1] << "," << fpga_.powerboard_I_list_[1] << ",";
        log_stream << fpga_.powerboard_V_list_[2] << "," << fpga_.powerboard_I_list_[2] << ",";
        log_stream << fpga_.powerboard_V_list_[3] << "," << fpga_.powerboard_I_list_[3] << ",";
        log_stream << fpga_.powerboard_V_list_[4] << "," << fpga_.powerboard_I_list_[4] << ",";
        log_stream << fpga_.powerboard_V_list_[5] << "," << fpga_.powerboard_I_list_[5] << ",";
        log_stream << fpga_.powerboard_V_list_[6] << "," << fpga_.powerboard_I_list_[6] << ",";
        log_stream << fpga_.powerboard_V_list_[7] << "," << fpga_.powerboard_I_list_[7] << ",";
        log_stream << fpga_.powerboard_V_list_[8] << "," << fpga_.powerboard_I_list_[8] << ",";
        log_stream << fpga_.powerboard_V_list_[9] << "," << fpga_.powerboard_I_list_[9] << ",";
        log_stream << fpga_.powerboard_V_list_[10] << "," << fpga_.powerboard_I_list_[10] << ",";
        log_stream << fpga_.powerboard_V_list_[11] << "," << fpga_.powerboard_I_list_[11] << std::endl;
    }
}

int main(int argc, char *argv[])
{
    signal(SIGINT, inthand);

    important_message("[FPGA Server] : Launched");

    if (argc == 3)
    {
        std::string s(argv[1]);
        if (s == "-t")
        {
            std::cout << "debug terminal output to " << argv[2] << std::endl;
            term = std::ofstream(argv[2], std::ios_base::out);
            term << "phi_vel_filt_r,"
                 << "phi_vel_filt_l,"
                 << "T_fb_r,"
                 << "T_fb_l,"
                 << "tau_inertia_r,"
                 << "tau_inertia_l,"
                 << "tau_friction_r,"
                 << "tau_friction_l,"
                 << "F_est_l2g[0],F_est_l2g[1],"
                 << "d_F_k[0],d_F_k[1],"
                 << "d_F_k_1[0],d_F_k_1[1],"
                 << "d_F_k_2[0],d_F_k_2[1],"
                 << "E_k[0],E_k[1],"
                 << "E_k_1[0],E_k_1[1],"
                 << "E_k_2[0],E_k_2[1],Xref[0],Xref[1],w1inv_00,w1inv_11\n";
        }
    }
    // Setup physical properties constant
    kinematics_setup();

    Corgi corgi;

    core::NodeHandler nh;
    core::ServiceServer<power_msg::PowerBoardStamped, power_msg::PowerBoardStamped> &power_srv =
        nh.serviceServer<power_msg::PowerBoardStamped, power_msg::PowerBoardStamped>("power/command", cb);

    core::Publisher<motor_msg::MotorStamped> &motor_pub = nh.advertise<motor_msg::MotorStamped>("motor/state");
    core::Subscriber<motor_msg::MotorStamped> &motor_sub = nh.subscribe<motor_msg::MotorStamped>("motor/command", 1000, motor_data_cb);

    core::Publisher<force_msg::LegForceStamped> &force_pub = nh.advertise<force_msg::LegForceStamped>("robot/force_state");
    core::Subscriber<force_msg::LegForceStamped> &force_sub = nh.subscribe<force_msg::LegForceStamped>("robot/force_command", 1000, force_data_cb);

    corgi.interruptHandler(power_srv, motor_sub, motor_pub, force_sub, force_pub);

    if (NiFpga_IsError(corgi.fpga_.status_))
    {
        std::cout << red << "[FPGA Server] Error! Exiting program. LabVIEW error code: " << corgi.fpga_.status_ << reset
                  << std::endl;
    }
    else
    {
        endwin();
        important_message("\n[FPGA Server] : Exit Safely");
    }

    return 0;
}

/* CAPTURE SYS STOP SIGNAL TO KILL PROCESS*/
void inthand(int signum)
{
    sys_stop = 1;
}
