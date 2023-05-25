#include "angle_convert.hpp"
#include "case_enum.hpp"
#include <fpga_server.hpp>

/* TCP node connection setup*/
// std::string local_ip = "169.254.254.89";
// std::string master_ip = "169.254.105.68";
// std::string local_ip = "192.168.50.148";

std::string master_ip = "127.0.0.1";
std::string local_ip = "127.0.0.1";
uint32_t master_port = 8888;

std::string command_topic = "/fpga/command";
std::string publish_topic = "/fpga/state";

motor_msg::MotorModule motor_msg_data;
int message_updated = 0;

void motor_module_cb(std::shared_ptr<motor_msg::MotorModule> msg)
{
    message_updated = 1;
    motor_msg_data = *msg;
}

Corgi::Corgi()
{
    stop_ = false;

    /* default value of interrupt*/
    main_irq_period_us_ = 500;
    can_irq_period_us_ = 800;

    /* initialize powerboard state */
    digital_switch_ = false;
    signal_switch_ = false;
    power_switch_ = false;

    max_timeout_cnt_ = 10;

    powerboard_state_.push_back(digital_switch_);
    powerboard_state_.push_back(signal_switch_);
    powerboard_state_.push_back(power_switch_);

    load_config_();

    ModeFsm fsm(&modules_list_);
    fsm_ = fsm;

    console_.init(&fpga_, &modules_list_, &powerboard_state_, &fsm_, &main_mtx_);

    fpga_.setIrqPeriod(main_irq_period_us_, can_irq_period_us_);

    MSG_Stream.open("/home/admin/fpga_driver/log/log.txt");
}

void Corgi::load_config_()
{
    yaml_node_ = YAML::LoadFile(CONFIG_PATH);

    master_ip = yaml_node_["Master_IP"].as<std::string>();
    master_port = yaml_node_["Master_port"].as<int>();
    local_ip = yaml_node_["Local_IP"].as<std::string>();

    command_topic = yaml_node_["Command_topic"].as<std::string>();
    publish_topic = yaml_node_["Publish_topic"].as<std::string>();

    if (yaml_node_["Scenario"].as<std::string>().compare("SingleModule") == 0)
        scenario_ = Scenario::SINGLE_MODULE;
    else
        scenario_ = Scenario::ROBOT;

    if (yaml_node_["Command_type"].as<std::string>().compare("phiR_phiL") == 0)
        cmd_type_ = Command_type::PHI_RL;
    else
        cmd_type_ = Command_type::THETA_BETA;

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
    // double Ifactor[12];
    // double Vfactor[12];

    std::cout << "PowerBoard Scaling Factor" << std::endl;
    for (auto f : Factors_node_)
    {
        fpga_.powerboard_Ifactor[idx_] = f["Current_Factor"].as<double>();
        fpga_.powerboard_Vfactor[idx_] = f["Voltage_Factor"].as<double>();
        std::cout << "Index " << idx_ << " Current Factor: " << fpga_.powerboard_Ifactor[idx_] << ", Voltage Factor: " << fpga_.powerboard_Vfactor[idx_] << std::endl;
        idx_++;
    }
}

void Corgi::interruptHandler(std::vector<core::Subscriber> &cmd_sub_, std::vector<core::Publisher> &state_pub_)
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
                mainLoop_(cmd_sub_, state_pub_);
                // Acknowledge IRQ to begin DMA acquisition
                NiFpga_MergeStatus(&fpga_.status_, NiFpga_AcknowledgeIrqs(fpga_.session_, irqsAsserted));
            }
            if (irqsAsserted & NiFpga_Irq_1)
            {
                /* TODO: do something if IRQ1 */
                /* Handling CAN-BUS communication */
                /* Interrupt enabled only in Motor mode*/
                if (fsm_.workingMode_ == Mode::MOTOR)
                {
                    canLoop_();
                }
                // Acknowledge IRQ to begin DMA acquisition
                NiFpga_MergeStatus(&fpga_.status_, NiFpga_AcknowledgeIrqs(fpga_.session_, irqsAsserted));
            }
        }
    }
}

void Corgi::mainLoop_(std::vector<core::Subscriber> &cmd_sub_, std::vector<core::Publisher> &state_pub_)
{
    fpga_.write_powerboard_(&powerboard_state_);
    fpga_.read_powerboard_data_();

    fsm_.runFsm();

    // Send Idle packet to achieve motor state while not in motor mode in order to update STM32 status
    if ((fsm_.workingMode_ == Mode::REST || fsm_.workingMode_ == Mode::SET_ZERO) && powerboard_state_.at(2) == true)
    {
        for (auto &mod : modules_list_)
        {
            if (mod.enable_)
            {
                CAN_txdata idle_txdata_;
                idle_txdata_.position_ = 0;
                idle_txdata_.torque_ = 0;
                idle_txdata_.KP_ = 0;
                idle_txdata_.KI_ = 0;
                idle_txdata_.KD_ = 0;

                mod.CAN_timeoutCheck();
                if (mod.CAN_module_timedout)
                {
                    timeout_cnt_++;
                }
                else
                {
                    timeout_cnt_ = 0;
                }

                if (timeout_cnt_ < max_timeout_cnt_)
                {
                    // modules_list_[i].io_.CAN_send_command(modules_list_[i].txdata_buffer_[0], modules_list_[i].txdata_buffer_[1]);
                    mod.io_.CAN_send_command(idle_txdata_, idle_txdata_);
                }

                // if (!mod.CAN_module_timedout)
                //     mod.io_.CAN_send_command(idle_txdata_, idle_txdata_);
            }
        }
    }

    // Communication with Node Architecture
    motor_msg::MotorModule motor_module;
    int index = 0;
    for (auto &mod : modules_list_)
    {
        if (mod.enable_)
        {
            mod.io_.CAN_recieve_feedback(&mod.rxdata_buffer_[0], &mod.rxdata_buffer_[1]);

            /* Pubish feedback data from Motors */
            // MSG_Stream << "mod.rxdata_buffer_[0].position_: " << mod.rxdata_buffer_[0].position_ << ", mod.rxdata_buffer_[1].position_: " << mod.rxdata_buffer_[0].position_ << std::endl;
            motor_module.phi_RL[0] = mod.rxdata_buffer_[0].position_; // phi R
            motor_module.phi_RL[1] = mod.rxdata_buffer_[1].position_; // phi L

            // double *tb_;
            double tb_[2] = {0, 0};
            getThetaBeta(tb_, mod.rxdata_buffer_[0].position_, mod.rxdata_buffer_[1].position_);
            // MSG_Stream << "Scenario: " << Scenario::SINGLE_MODULE << std::endl;

            if (scenario_ == Scenario::SINGLE_MODULE)
            {
                // MSG_Stream << "motor_module.theta_beta[0]: " << tb_[0] << ", mod.rxdata_buffer_[1].position_: " << tb_[1] << std::endl;
                motor_module.theta_beta[0] = tb_[0]; // theta
                motor_module.theta_beta[1] = tb_[1]; // beta
            }
            else
            {
                /* Special Case for Module A and D in Robot Scenario [Module's beta frame should be inverted]*/
                motor_module.theta_beta[0] = tb_[0]; // theta
                if (index == 0 || index == 3)
                {
                    // MSG_Stream << "motor_module.theta_beta[0]: " << tb_[0] << ", mod.rxdata_buffer_[1].position_: " << tb_[1] << std::endl;
                    // MSG_Stream << "raw: " << tb_[1] << "processed: " << motor_module.theta_beta[1];
                    motor_module.theta_beta[1] = -1 * tb_[1]; // beta
                }
                else
                    motor_module.theta_beta[1] = tb_[1]; // beta
                // MSG_Stream << "motor_module.theta_beta[0]: " << tb_[0] << ", mod.rxdata_buffer_[1].position_: " << tb_[1] << std::endl;
            }

            // for (int i = 0; i < 4; i++)
            // {
            // MSG_Stream << "mod " << i << " rx timeout" << modules_list_[i].CAN_rx_timedout_[0] << modules_list_[i].CAN_rx_timedout_[1] << std::endl;
            // MSG_Stream << "mod " << i << " tx timeout" << modules_list_[i].CAN_tx_timedout_[0] << modules_list_[i].CAN_tx_timedout_[1] << std::endl;
            // }
            // MSG_Stream << "---" << std::endl;

            motor_module.twist[0] = mod.rxdata_buffer_[0].velocity_; // velocity R
            motor_module.twist[1] = mod.rxdata_buffer_[1].velocity_; // velocity L
            motor_module.torque[0] = mod.rxdata_buffer_[0].torque_;  // torque R
            motor_module.torque[1] = mod.rxdata_buffer_[1].torque_;  // torque L
            state_pub_[index].publish(motor_module);

            /* Subscribe command from other nodes */
            // initialize msg
            motor_msg_data.phi_RL[0] = 0;
            motor_msg_data.phi_RL[1] = 0;
            motor_msg_data.twist[0] = 0;
            motor_msg_data.twist[1] = 0;
            motor_msg_data.torque[0] = 0;
            motor_msg_data.torque[1] = 0;
            motor_msg_data.pid[0] = 0;
            motor_msg_data.pid[1] = 0;
            motor_msg_data.pid[2] = 0;
            motor_msg_data.pid[3] = 0;
            motor_msg_data.pid[4] = 0;
            motor_msg_data.pid[5] = 0;
            // update
            cmd_sub_[index].spinOnce(motor_module_cb);

            if (message_updated)
            {
                if (cmd_type_ == Command_type::THETA_BETA)
                {
                    // double *phi;
                    double phi[2] = {0, 0};
                    // Full Robot experiment scenario
                    if (scenario_ == Scenario::ROBOT)
                    {
                        // Special Case for module A D should be inverted
                        if (index == 0 || index == 3)
                        {
                            getPhiVector(phi, motor_msg_data.theta_beta[0], -1 * motor_msg_data.theta_beta[1]);
                        }
                        else
                        {
                            getPhiVector(phi, motor_msg_data.theta_beta[0], motor_msg_data.theta_beta[1]);
                        }
                    }
                    // Single Module experiment scenario
                    else
                    {
                        getPhiVector(phi, motor_msg_data.theta_beta[0], motor_msg_data.theta_beta[1]);
                    }
                    mod.txdata_buffer_[0].position_ = phi[0];
                    mod.txdata_buffer_[1].position_ = phi[1];
                }
                else
                {
                    // Command Type Phi_R Phi_L
                    mod.txdata_buffer_[0].position_ = motor_msg_data.phi_RL[0];
                    mod.txdata_buffer_[1].position_ = motor_msg_data.phi_RL[1];
                }

                mod.txdata_buffer_[0].torque_ = motor_msg_data.torque[0];
                mod.txdata_buffer_[1].torque_ = motor_msg_data.torque[1];
                mod.txdata_buffer_[0].KP_ = motor_msg_data.pid[0];
                mod.txdata_buffer_[0].KI_ = motor_msg_data.pid[1];
                mod.txdata_buffer_[0].KD_ = motor_msg_data.pid[2];
                mod.txdata_buffer_[1].KP_ = motor_msg_data.pid[3];
                mod.txdata_buffer_[1].KI_ = motor_msg_data.pid[4];
                mod.txdata_buffer_[1].KD_ = motor_msg_data.pid[5];
            }
            message_updated = 0;
        }
        index++;
    }
}

void Corgi::canLoop_()
{
    for (int i = 0; i < 4; i++)
    {
        if (modules_list_[i].enable_)
        {
            if (modules_list_[i].rxdata_buffer_[0].mode_ == Mode::MOTOR && modules_list_[i].rxdata_buffer_[1].mode_ == Mode::MOTOR)
            {
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
                }
            }
        }
    }
}

int main()
{
    signal(SIGINT, inthand);

    important_message("[FPGA Server] : Launched");
    Corgi corgi;

    core::NodeHandler nh(master_ip, master_port, local_ip);
    std::vector<core::Subscriber> cmd_subs;
    std::vector<core::Publisher> state_pubs;
    for (int i = 0; i < 4; i++)
    {
        cmd_subs.push_back(nh.subscriber(command_topic + std::to_string(i + 1)));
        usleep(100000);
        state_pubs.push_back(nh.publisher(publish_topic + std::to_string(i + 1)));
        usleep(100000);
    }
    boost::thread node_service(boost::bind(&boost::asio::io_service::run, &core::node_ios));

    corgi.interruptHandler(cmd_subs, state_pubs);

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
