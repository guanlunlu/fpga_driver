#include <leg_module.hpp>

LegModule::LegModule(std::string _label, YAML::Node _config, NiFpga_Status _status, NiFpga_Session _fpga_session)
{
    label_ = _label;
    config_ = _config;
    enable_ = false;

    load_config();

    CAN_tx_timedout_[0] = false;
    CAN_tx_timedout_[1] = false;
    CAN_rx_timedout_[0] = false;
    CAN_rx_timedout_[1] = false;

    txdata_buffer_[0].position_ = 0;
    txdata_buffer_[0].torque_ = 0;
    txdata_buffer_[0].KP_ = motors_list_[0].kp_;
    txdata_buffer_[0].KI_ = motors_list_[0].ki_;
    txdata_buffer_[0].KD_ = motors_list_[0].kd_;

    txdata_buffer_[1].position_ = 0;
    txdata_buffer_[1].torque_ = 0;
    txdata_buffer_[1].KP_ = motors_list_[1].kp_;
    txdata_buffer_[1].KI_ = motors_list_[1].ki_;
    txdata_buffer_[1].KD_ = motors_list_[1].kd_;

    rxdata_buffer_[0].mode_ = Mode::REST;
    rxdata_buffer_[0].mode_state_ = _REST_MODE;
    rxdata_buffer_[0].position_ = 0;
    rxdata_buffer_[0].torque_ = 0;
    rxdata_buffer_[0].velocity_ = 0;
    rxdata_buffer_[0].calibrate_finish_ = 0;
    rxdata_buffer_[0].CAN_id_ = 0;
    rxdata_buffer_[0].version_ = 0;

    rxdata_buffer_[1].mode_ = Mode::REST;
    rxdata_buffer_[1].mode_state_ = _REST_MODE;
    rxdata_buffer_[1].position_ = 0;
    rxdata_buffer_[1].torque_ = 0;
    rxdata_buffer_[1].velocity_ = 0;
    rxdata_buffer_[1].calibrate_finish_ = 0;
    rxdata_buffer_[1].CAN_id_ = 0;
    rxdata_buffer_[1].version_ = 0;

    io_ = ModuleIO(_status, _fpga_session, CAN_port_, motors_list_);
    CAN_first_transmit_ = true;

    /* setup motors' CAN ID, port selection and timeout_us */
    io_.CAN_setup(CAN_timeout_us);
}

void LegModule::load_config()
{
    Motor motor_f;
    Motor motor_h;
    CAN_timeout_us = config_["CAN_Timeout_us"].as<int>();

    // load configuration from yaml file
    std::cout << "[ " << label_ << " Configuration ]" << std::endl;
    enable_ = config_[label_]["Enable"].as<int>();
    CAN_port_ = config_[label_]["CAN_PORT"].as<std::string>();
    // Motor F setup
    motor_f.fw_version_ = config_[label_]["Motor_F"]["FW_Version"].as<int>();
    motor_f.CAN_ID_ = config_[label_]["Motor_F"]["CAN_ID"].as<int>();
    motor_f.kp_ = config_[label_]["Motor_F"]["KP"].as<double>();
    motor_f.ki_ = config_[label_]["Motor_F"]["KI"].as<double>();
    motor_f.kd_ = config_[label_]["Motor_F"]["KD"].as<double>();
    motor_f.torque_ff_ = config_[label_]["Motor_F"]["Torque_Feedfoward"].as<int>();

    // Motor H setup
    motor_h.fw_version_ = config_[label_]["Motor_H"]["FW_Version"].as<int>();
    motor_h.CAN_ID_ = config_[label_]["Motor_H"]["CAN_ID"].as<int>();
    motor_h.kp_ = config_[label_]["Motor_H"]["KP"].as<double>();
    motor_h.ki_ = config_[label_]["Motor_H"]["KI"].as<double>();
    motor_h.kd_ = config_[label_]["Motor_H"]["KD"].as<double>();
    motor_h.torque_ff_ = config_[label_]["Motor_H"]["Torque_Feedfoward"].as<int>();

    motors_list_.push_back(motor_f);
    motors_list_.push_back(motor_h);

    std::cout << "CAN PORT: " << config_[label_]["CAN_PORT"].as<std::string>() << std::endl;

    std::cout << "Motor_F: " << std::endl;
    std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor_f.fw_version_ << std::endl;
    std::cout << std::setw(14) << "  CAN_ID: " << std::setw(13) << motor_f.CAN_ID_ << std::endl;
    std::cout << std::setw(14) << "  KP: " << std::setw(13) << motor_f.kp_ << std::endl;
    std::cout << std::setw(14) << "  KI: " << std::setw(13) << motor_f.ki_ << std::endl;
    std::cout << std::setw(14) << "  KD: " << std::setw(13) << motor_f.kd_ << std::endl;
    std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motor_f.torque_ff_ << std::endl;
    std::cout << std::setw(14) << "---------------------------" << std::endl;

    std::cout << "Motor_H: " << std::endl;
    std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor_h.fw_version_ << std::endl;
    std::cout << std::setw(14) << "  CAN_ID: " << std::setw(13) << motor_h.CAN_ID_ << std::endl;
    std::cout << std::setw(14) << "  KP: " << std::setw(13) << motor_h.kp_ << std::endl;
    std::cout << std::setw(14) << "  KI: " << std::setw(13) << motor_h.ki_ << std::endl;
    std::cout << std::setw(14) << "  KD: " << std::setw(13) << motor_h.kd_ << std::endl;
    std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motor_h.torque_ff_ << std::endl;
    std::cout << std::setw(14) << "---------------------------" << std::endl;
}
