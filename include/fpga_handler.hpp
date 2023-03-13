#ifndef __FPGAHANDLER_H
#define __FPGAHANDLER_H

#include "NiFpga.h"
#include <iostream>
#include <vector>
#include <functional>
#include <boost/bind.hpp>
#include <dlfcn.h>
#include <NiFpga_FPGA_CANBus_IMU_4module_IRQ.h>
#include <can_packet.h>
#include <color.hpp>
#include <signal.h>
#include <msg.hpp>

class ModuleIO
{
  public:
    ModuleIO(NiFpga_Status status_, NiFpga_Session fpga_session_, std::string CAN_port_,
             std::vector<Motor> motors_list);

    ModuleIO()
    {
    }

    NiFpga_Status status_;
    NiFpga_Session fpga_session_;
    std::vector<Motor> motors_list_;

    int CAN_timeout_us_;

    


    NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32 r_CAN_id1_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32 r_CAN_id2_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayBool r_port_select_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayBoolSize r_port_select_size_;
    // tx buffer
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8 r_tx_buf_id1_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8 r_tx_buf_id2_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8Size r_tx_buf_size_;
    // rx buffer
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8 r_rx_buf_id1_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8 r_rx_buf_id2_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8Size r_rx_buf_size_;

    NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlBool r_CAN_transmit_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool r_CAN_complete_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool r_CAN_success_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorI16 r_CAN_complete_counter_;

    NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool r_tx_timeout_id1_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool r_tx_timeout_id2_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool r_rx_timeout_id1_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorBool r_rx_timeout_id2_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlU32 r_timeout_us_;

    

    // read write function
    void write_CAN_id_(uint32_t id1, uint32_t id2);
    void write_port_select_(const NiFpga_Bool* array);

    void write_tx_data_(const uint8_t* tx_arr1, const uint8_t* tx_arr2);
    void read_rx_data_(uint8_t* rx_arr1, uint8_t* rx_arr2);

    void write_CAN_transmit_(NiFpga_Bool value);
    NiFpga_Bool read_CAN_complete_();
    NiFpga_Bool read_CAN_success_();
    int16_t read_CAN_complete_counter_();

    void write_timeout_us_(uint32_t value);
    NiFpga_Bool read_tx_id1_timeout_();
    NiFpga_Bool read_tx_id2_timeout_();
    NiFpga_Bool read_rx_id1_timeout_();
    NiFpga_Bool read_rx_id2_timeout_();

    void CAN_setup(int timeout_us);
    void CAN_set_mode(Mode mode);
    void CAN_send_command(CAN_txdata txdata_id1, CAN_txdata txdata_id2);
    void CAN_recieve_feedback(CAN_rxdata* rxdata_id1, CAN_rxdata* rxdata_id2);

    void CAN_encode(uint8_t (&txmsg)[8], CAN_txdata txdata);
    void CAN_decode(uint8_t (&rxmsg)[8], CAN_rxdata* rxdata);

    // data conversion for CAN-bus
    int float_to_uint(float x, float x_min, float x_max, int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

class FpgaHandler
{
  public:
    FpgaHandler();
    ~FpgaHandler();

    void setIrqPeriod(int main_loop_period, int can_loop_period);

    NiFpga_Session session_;
    NiFpga_Status status_;
    // Fpga interrupt request
    NiFpga_IrqContext irqContext_;


    // powerboard 
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlBool w_pb_digital_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlBool w_pb_signal_;
    NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlBool w_pb_power_;
    
    void write_powerboard_(std::vector<bool>* powerboard_state_);
};

#endif
