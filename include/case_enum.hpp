#ifndef __ENUME_H
#define __ENUME_H

enum class Module_ID
{
    LF,
    LH,
    RF,
    RH
};

enum class Scenario
{
    ROBOT,
    SINGLE_MODULE
};

enum class Command_type
{
    PHI_RL,
    THETA_BETA
};

#endif