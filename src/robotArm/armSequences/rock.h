#ifndef ROBOT_ARM_SEQUENCES_ROCK_H
#define ROBOT_ARM_SEQUENCES_ROCK_H

#include "../ArmController2.h"
#include "../taskManager.h"

enum class RockSide
{
    LEFT,
    RIGHT
};

static constexpr ArmPose LEFT_ROCK_GRAB_OPEN =
{
    185, 230, 220, 225, 25
};

static constexpr ArmPose LEFT_ROCK_GRAB_CLOSED =
{
    185, 230, 220, 225, 40
};

static constexpr ArmPose RIGHT_ROCK_GRAB_OPEN =
{
    85, 230, 220, 225, 25
};

static constexpr ArmPose RIGHT_ROCK_GRAB_CLOSED =
{
    85, 230, 220, 225, 40
};


static constexpr ArmPose ROCK_OVER_POST =
{
    130, 175, 25, 140, 40
};

static constexpr ArmPose ROCK_PLACE =
{
    130, 175, 25, 140, 25
};

static constexpr ArmPose ROCK_RETRACT =
{
    130, 70, 235, 35, 25
};

#endif // ROBOT_ARM_SEQUENCES_ROCK_H
/*
#define HOME_BASE     130
#define HOME_SHOULDER 70
#define HOME_ELBOW    235
#define HOME_WRIST    35
#define HOME_CLAW      25 */