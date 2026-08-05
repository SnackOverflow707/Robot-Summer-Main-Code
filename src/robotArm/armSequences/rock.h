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
    185, 225, 220, 235, 25
};

static constexpr ArmPose LEFT_ROCK_GRAB_CLOSED =
{
    185, 225, 220, 235, 15
};

static constexpr ArmPose RIGHT_ROCK_GRAB_OPEN =
{
    85, 225, 220, 235, 25
};

static constexpr ArmPose RIGHT_ROCK_GRAB_CLOSED =
{
    85, 225, 220, 235, 35
};

static constexpr ArmPose ROCK_LIFT =
{
    130, 70, 235, 35, 35
};

static constexpr ArmPose ROCK_OVER_POST =
{
    135, 185, 0, 130, 35
};

static constexpr ArmPose ROCK_PLACE =
{
    135, 185, 0, 130, 15
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