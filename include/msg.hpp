#ifndef __MSG_H
#define __MSG_H
#include <mode.hpp>
#include <vector>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/vector.hpp>
#include "cereal/types/utility.hpp"
#include <math.h>
#include <string>

typedef struct Motor
{
    int CAN_ID_;
    int fw_version_;
    double kp_;
    double ki_;
    double kd_;
    double torque_ff_;

    double calibration_bias;
} Motor;

// transmitted to SBRIO
typedef struct CAN_txdata
{
    float position_;
    float torque_;
    float KP_;
    float KI_;
    float KD_;
} CAN_txdata;

typedef struct CAN_rxdata
{
    int CAN_id_;
    float position_;
    float velocity_;
    float torque_;
    int version_;
    int calibrate_finish_;
    int mode_state_;
    Mode mode_;
} CAN_rxdata;

class Module
{
public:
    std::vector<CAN_txdata> txdata_;
};

#endif
