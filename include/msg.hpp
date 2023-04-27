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

    Mode mode_fb_;
    int version_fb_;
    bool calibrated_fb_;

    double position_fb_;
    double current_fb_;
    double velocity_fb_;

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

class FpgaCmdMsg
{
public:
    bool power_on_ = false;
    bool digital_on_ = false;
    bool signal_on_ = true;
    bool stop_ = false;

    Mode mode_;

    std::vector<Module> modules_;

    template <class Archive>
    void serialize(Archive &ar)
    {
        ar(power_on_, digital_on_, signal_on_, stop_);
    }
};

#endif
