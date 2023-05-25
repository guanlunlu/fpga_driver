#include <fpga_handler.hpp>
#include <ncurses.h>
#include <curses.h>

ModuleIO::ModuleIO(NiFpga_Status _status, NiFpga_Session _fpga_session, std::string CAN_port_,
                   std::vector<Motor> *motors_list)
{
    status_ = _status;
    fpga_session_ = _fpga_session;
    motors_list_ = motors_list;

    CAN_timeout_us_ = 500;

    if (CAN_port_ == "MOD1CAN0")
    {
        r_CAN_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_Mod1CAN0ID1;
        r_CAN_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_Mod1CAN0ID2;

        r_port_select_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayBool_Mod1CAN0Select;
        r_port_select_size_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayBoolSize_Mod1CAN0Select;

        r_tx_buf_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8_Mod1CAN0ID1TXData;
        r_tx_buf_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8_Mod1CAN0ID2TXData;
        r_tx_buf_size_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8Size_Mod1CAN0ID1TXData;

        r_rx_buf_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8_Mod1CAN0ID1RXData;
        r_rx_buf_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8_Mod1CAN0ID2RXData;
        r_rx_buf_size_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8Size_Mod1CAN0ID1RXData;

        r_CAN_transmit_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlBool_MOD1CAN0Transmit;
        r_CAN_complete_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod1CAN0Complete;
        r_CAN_success_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod1CAN0success;
        r_CAN_complete_counter_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorI16_Mod1CAN0CompleteCounter;

        r_tx_timeout_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod1CAN0ID1TXTimedout;
        r_tx_timeout_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod1CAN0ID2TXTimedout;
        r_rx_timeout_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod1CAN0ID1RXTimedout;
        r_rx_timeout_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod1CAN0ID2RXTimedout;

        r_timeout_us_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_Mod1CAN0RXTimoutus;
    }
    else if (CAN_port_ == "MOD1CAN1")
    {
        r_CAN_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_Mod1CAN1ID1;
        r_CAN_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_Mod1CAN1ID2;

        r_port_select_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayBool_Mod1CAN1Select;
        r_port_select_size_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayBoolSize_Mod1CAN1Select;

        r_tx_buf_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8_Mod1CAN1ID1TXData;
        r_tx_buf_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8_Mod1CAN1ID2TXData;
        r_tx_buf_size_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8Size_Mod1CAN1ID1TXData;

        r_rx_buf_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8_Mod1CAN1ID1RXData;
        r_rx_buf_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8_Mod1CAN1ID2RXData;
        r_rx_buf_size_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8Size_Mod1CAN1ID1RXData;

        r_CAN_transmit_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlBool_MOD1CAN1Transmit;
        r_CAN_complete_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod1CAN1Complete;
        r_CAN_success_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod1CAN1success;
        r_CAN_complete_counter_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorI16_Mod1CAN1CompleteCounter;

        r_tx_timeout_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod1CAN1ID1TXTimedout;
        r_tx_timeout_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod1CAN1ID2TXTimedout;
        r_rx_timeout_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod1CAN1ID1RXTimedout;
        r_rx_timeout_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod1CAN1ID2RXTimedout;

        r_timeout_us_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_Mod1CAN1RXTimoutus;
    }
    else if (CAN_port_ == "MOD2CAN0")
    {
        r_CAN_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_Mod2CAN0ID1;
        r_CAN_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_Mod2CAN0ID2;

        r_port_select_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayBool_Mod2CAN0Select;
        r_port_select_size_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayBoolSize_Mod2CAN0Select;

        r_tx_buf_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8_Mod2CAN0ID1TXData;
        r_tx_buf_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8_Mod2CAN0ID2TXData;
        r_tx_buf_size_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8Size_Mod2CAN0ID1TXData;

        r_rx_buf_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8_Mod2CAN0ID1RXData;
        r_rx_buf_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8_Mod2CAN0ID2RXData;
        r_rx_buf_size_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8Size_Mod2CAN0ID1RXData;

        r_CAN_transmit_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlBool_MOD2CAN0Transmit;
        r_CAN_complete_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod2CAN0Complete;
        r_CAN_success_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod2CAN0success;
        r_CAN_complete_counter_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorI16_Mod2CAN0CompleteCounter;

        r_tx_timeout_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod2CAN0ID1TXTimedout;
        r_tx_timeout_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod2CAN0ID2TXTimedout;
        r_rx_timeout_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod2CAN0ID1RXTimedout;
        r_rx_timeout_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod2CAN0ID2RXTimedout;

        r_timeout_us_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_Mod2CAN0RXTimoutus;
    }
    else if (CAN_port_ == "MOD2CAN1")
    {
        r_CAN_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_Mod2CAN1ID1;
        r_CAN_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_Mod2CAN1ID2;

        r_port_select_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayBool_Mod2CAN1Select;
        r_port_select_size_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayBoolSize_Mod2CAN1Select;

        r_tx_buf_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8_Mod2CAN1ID1TXData;
        r_tx_buf_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8_Mod2CAN1ID2TXData;
        r_tx_buf_size_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8Size_Mod2CAN1ID1TXData;

        r_rx_buf_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8_Mod2CAN1ID1RXData;
        r_rx_buf_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8_Mod2CAN1ID2RXData;
        r_rx_buf_size_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8Size_Mod2CAN1ID1RXData;

        r_CAN_transmit_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlBool_MOD2CAN1Transmit;
        r_CAN_complete_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod2CAN1Complete;
        r_CAN_success_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod2CAN1success;
        r_CAN_complete_counter_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorI16_Mod2CAN1CompleteCounter;

        r_tx_timeout_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod2CAN1ID1TXTimedout;
        r_tx_timeout_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod2CAN1ID2TXTimedout;
        r_rx_timeout_id1_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod2CAN1ID1RXTimedout;
        r_rx_timeout_id2_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool_Mod2CAN1ID2RXTimedout;

        r_timeout_us_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_Mod2CAN1RXTimoutus;
    }
    else
    {
        std::cout << "[ERROR] CAN_PORT CONFIG ERROR !" << std::endl;
    }
}

// Write FPGA status

void ModuleIO::write_CAN_id_(uint32_t id1, uint32_t id2)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_CAN_id1_, id1));
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_CAN_id2_, id2));
}

void ModuleIO::write_port_select_(const NiFpga_Bool *array)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteArrayBool(fpga_session_, r_port_select_, array, r_port_select_size_));
}

void ModuleIO::write_tx_data_(const uint8_t *tx_arr1, const uint8_t *tx_arr2)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteArrayU8(fpga_session_, r_tx_buf_id1_, tx_arr1, r_tx_buf_size_));
    NiFpga_MergeStatus(&status_, NiFpga_WriteArrayU8(fpga_session_, r_tx_buf_id2_, tx_arr2, r_tx_buf_size_));
}

void ModuleIO::write_CAN_transmit_(NiFpga_Bool value)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(fpga_session_, r_CAN_transmit_, value));
}

void ModuleIO::write_timeout_us_(uint32_t value)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_timeout_us_, value));
}

// Read FPGA status

void ModuleIO::read_rx_data_(uint8_t *rx_arr1, uint8_t *rx_arr2)
{
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU8(fpga_session_, r_rx_buf_id1_, rx_arr1, r_rx_buf_size_));
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU8(fpga_session_, r_rx_buf_id2_, rx_arr2, r_rx_buf_size_));
}

NiFpga_Bool ModuleIO::read_CAN_complete_()
{
    NiFpga_Bool complete = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_CAN_complete_, &complete));
    return complete;
}

NiFpga_Bool ModuleIO::read_CAN_success_()
{
    NiFpga_Bool success = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_CAN_success_, &success));
    return success;
}

int16_t ModuleIO::read_CAN_complete_counter_()
{
    int16_t count = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadI16(fpga_session_, r_CAN_complete_counter_, &count));
    return count;
}

NiFpga_Bool ModuleIO::read_tx_id1_timeout_()
{
    NiFpga_Bool id1_timeout = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_tx_timeout_id1_, &id1_timeout));
    return id1_timeout;
}

NiFpga_Bool ModuleIO::read_tx_id2_timeout_()
{
    NiFpga_Bool id2_timeout = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_tx_timeout_id2_, &id2_timeout));
    return id2_timeout;
}

NiFpga_Bool ModuleIO::read_rx_id1_timeout_()
{
    NiFpga_Bool id1_timeout = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_rx_timeout_id1_, &id1_timeout));
    return id1_timeout;
}

NiFpga_Bool ModuleIO::read_rx_id2_timeout_()
{
    NiFpga_Bool id2_timeout = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_rx_timeout_id2_, &id2_timeout));
    return id2_timeout;
}

void ModuleIO::CAN_setup(int timeout_us)
{
    write_CAN_id_(motors_list_->at(0).CAN_ID_, motors_list_->at(1).CAN_ID_);

    /* select two port to transceive */
    NiFpga_Bool _bool_arr[2] = {1, 1};
    write_port_select_(_bool_arr);

    write_timeout_us_(timeout_us);
}

void ModuleIO::CAN_set_mode(Mode mode)
{
    uint8_t tx_msg[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    switch (mode)
    {
    case Mode::SET_ZERO:
        tx_msg[7] = 0xFE;
        break;
    case Mode::REST:
        tx_msg[7] = 0xFD;
        break;
    case Mode::HALL_CALIBRATE:
        tx_msg[7] = 0xFA;
        break;
    case Mode::MOTOR:
        tx_msg[7] = 0xFC;
        break;
    }
    write_tx_data_(tx_msg, tx_msg);
    write_CAN_transmit_(1);
}

void ModuleIO::CAN_send_command(CAN_txdata txdata_id1, CAN_txdata txdata_id2)
{
    uint8_t txmsg_id1[8];
    uint8_t txmsg_id2[8];
    CAN_txdata txdata1_biased;
    CAN_txdata txdata2_biased;

    txdata1_biased.position_ = txdata_id1.position_ + motorR_bias;
    txdata1_biased.torque_ = txdata_id1.torque_;
    txdata1_biased.KP_ = txdata_id1.KP_;
    txdata1_biased.KI_ = txdata_id1.KI_;
    txdata1_biased.KD_ = txdata_id1.KD_;

    txdata2_biased.position_ = txdata_id2.position_ + motorL_bias;
    txdata2_biased.torque_ = txdata_id2.torque_;
    txdata2_biased.KP_ = txdata_id2.KP_;
    txdata2_biased.KI_ = txdata_id2.KI_;
    txdata2_biased.KD_ = txdata_id2.KD_;

    // std::cout << "R bias : " << motorR_bias << std::endl;
    // std::cout << "L bias : " << motorL_bias << std::endl;

    CAN_encode(txmsg_id1, txdata1_biased);
    CAN_encode(txmsg_id2, txdata2_biased);

    // CAN_encode(txmsg_id1, txdata_id1);
    // CAN_encode(txmsg_id2, txdata_id2);

    write_tx_data_(txmsg_id1, txmsg_id2);
    write_CAN_transmit_(1);
}

void ModuleIO::CAN_recieve_feedback(CAN_rxdata *rxdata_id1, CAN_rxdata *rxdata_id2)
{
    uint8_t rxmsg_id1[8];
    uint8_t rxmsg_id2[8];
    read_rx_data_(rxmsg_id1, rxmsg_id2);
    CAN_decode(rxmsg_id1, rxdata_id1);
    CAN_decode(rxmsg_id2, rxdata_id2);

    // std::cout << "CAN_recieve_feedback" << std::endl;

    rxdata_id1->position_ -= motorR_bias;
    rxdata_id2->position_ -= motorL_bias;
}

// pack CAN data

void ModuleIO::CAN_encode(uint8_t (&txmsg)[8], CAN_txdata txdata)
{
    int pos_int, torque_int, KP_int, KI_int, KD_int;
    pos_int = float_to_uint(txdata.position_, P_CMD_MIN, P_CMD_MAX, 16);
    KP_int = float_to_uint(txdata.KP_, KP_MIN, KP_MAX, 12);
    KI_int = float_to_uint(txdata.KI_, KI_MIN, KI_MAX, 12);
    KD_int = float_to_uint(txdata.KD_, KD_MIN, KD_MAX, 12);
    torque_int = float_to_uint(txdata.torque_, T_MIN, T_MAX, 12);

    txmsg[0] = pos_int >> 8;
    txmsg[1] = pos_int & 0xFF;
    txmsg[2] = KP_int >> 4;
    txmsg[3] = ((KP_int & 0x0F) << 4) | (KI_int >> 8);
    txmsg[4] = KI_int & 0xFF;
    txmsg[5] = KD_int >> 4;
    txmsg[6] = ((KD_int & 0x0F) << 4) | (torque_int >> 8);
    txmsg[7] = torque_int & 0xFF;
}

void ModuleIO::CAN_decode(uint8_t (&rxmsg)[8], CAN_rxdata *rxdata)
{
    int pos_raw, vel_raw, torque_raw, ver_raw, cal_raw, mode_raw;

    // CAN bus ID, 8-bit
    // Position Measurement, 16-bit
    // Velocity Measurement, 12-bit
    // Torque Estimated, 12-bit
    // Version, 4-bit
    // Hall Calibrate status, 4-bit
    // Mode, 8-bit

    rxdata->CAN_id_ = (int)rxmsg[0];
    pos_raw = ((int)(rxmsg[1]) << 8) | rxmsg[2];
    vel_raw = (((int)(rxmsg[3]) << 4)) | ((rxmsg[4] & 0xF0) >> 4);
    torque_raw = ((((int)rxmsg[4]) & 0x0F) << 8) | rxmsg[5];
    ver_raw = ((int)(rxmsg[6] >> 4));
    cal_raw = ((int)(rxmsg[6] & 0x0F));
    mode_raw = ((int)(rxmsg[7]));

    rxdata->position_ = uint_to_float(pos_raw, P_FB_MIN, P_FB_MAX, 16);
    rxdata->velocity_ = uint_to_float(vel_raw, V_MIN, V_MAX, 12);
    rxdata->torque_ = uint_to_float(torque_raw, T_MIN, T_MAX, 12);
    rxdata->version_ = ver_raw;
    rxdata->calibrate_finish_ = cal_raw;
    rxdata->mode_state_ = mode_raw;

    if (mode_raw == _SET_ZERO)
    {
        rxdata->mode_ = Mode::SET_ZERO;
    }
    else if (mode_raw == _MOTOR_MODE)
    {
        rxdata->mode_ = Mode::MOTOR;
    }
    else if (mode_raw == _HALL_CALIBRATE)
    {
        rxdata->mode_ = Mode::HALL_CALIBRATE;
    }
    else if (mode_raw == _REST_MODE)
    {
        rxdata->mode_ = Mode::REST;
    }
}

int ModuleIO::float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float ModuleIO::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void ModuleIO::set_calibration_bias(double mtrR_bias, double mtrL_bias)
{
    motorR_bias = mtrR_bias;
    motorL_bias = mtrL_bias;
}

FpgaHandler::FpgaHandler()
{
    status_ = NiFpga_Initialize();
    important_message("[FPGA Handler] Fpga Initialized");

    NiFpga_MergeStatus(&status_, NiFpga_Open(NiFpga_FPGA_CANBus_IMU_4module_IRQ_Bitfile,
                                             NiFpga_FPGA_CANBus_IMU_4module_IRQ_Signature, "RIO0", 0, &session_));
    important_message("[FPGA Handler] Session opened");

    NiFpga_MergeStatus(&status_, NiFpga_ReserveIrqContext(session_, &irqContext_));
    important_message("[FPGA Handler] IRQ reserved");

    w_pb_digital_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlBool_Digital;
    w_pb_signal_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlBool_Signal;
    w_pb_power_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlBool_Power;

    r_powerboard_data_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU16_Data;
    size_powerboard_data_ = NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU16Size_Data;

    for (int i = 0; i < 12; i++)
    {
        powerboard_V_list_[i] = 0;
        powerboard_I_list_[i] = 0;
    }
}

FpgaHandler::~FpgaHandler()
{
    /* unreserve IRQ status to prevent memory leaks */
    NiFpga_MergeStatus(&status_, NiFpga_UnreserveIrqContext(session_, &irqContext_));

    /* Close the session */
    NiFpga_MergeStatus(&status_, NiFpga_Close(session_, 0));
    important_message("[FPGA Handler] Session Closed");

    NiFpga_MergeStatus(&status_, NiFpga_Finalize());
    important_message("[FPGA Handler] Fpga Finalized");
}

void FpgaHandler::setIrqPeriod(int main_loop_p, int can_loop_p)
{
    /* Set up interrupt period (microsecond) */
    /* IRQ 0 */
    NiFpga_MergeStatus(
        &status_, NiFpga_WriteU32(session_, NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_IRQ0_period_us, main_loop_p));

    /* IRQ 1 */
    NiFpga_MergeStatus(
        &status_, NiFpga_WriteU32(session_, NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32_IRQ1_period_us, can_loop_p));
}

void FpgaHandler::write_powerboard_(std::vector<bool> *powerboard_state_)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_digital_, powerboard_state_->at(0)));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_signal_, powerboard_state_->at(1)));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_power_, powerboard_state_->at(2)));
}

void FpgaHandler::read_powerboard_data_()
{
    uint16_t rx_arr[24];
    // uint16_t *rx_arr = new uint16_t[24];
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU16(session_, NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU16_Data, rx_arr, NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU16Size_Data));

    for (int i = 0; i < 24; i++)
    {
        if (i % 2 == 0)
        {
            powerboard_I_list_[i / 2] = rx_arr[i] * powerboard_Ifactor[i / 2];
            // printf("Ifactor = %f\n", powerboard_Ifactor[i / 2]);
        }
        if (i % 2 == 1)
        {
            powerboard_V_list_[(i - 1) / 2] = rx_arr[i] * powerboard_Vfactor[(i - 1) / 2];
        }
    }
}