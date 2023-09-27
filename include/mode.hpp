#ifndef __MODE_H
#define __MODE_H

#define _REST_MODE 0
#define _HALL_CALIBRATE 1 // hall sensor calibration
#define _MOTOR_MODE 2
#define _SET_ZERO 3

enum class Mode
{
    REST,
    MOTOR,
    SET_ZERO,
    HALL_CALIBRATE
};

enum class Behavior
{
    SET_THETA,
    TCP_SLAVE,
    CUSTOM_1,
    CUSTOM_2,
    CUSTOM_3
};

#endif