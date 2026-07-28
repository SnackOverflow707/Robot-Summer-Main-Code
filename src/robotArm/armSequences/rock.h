#ifndef ROCK_H
#define ROCK_H

#include "../ArmController2.h"
#include "../taskManager.h"

#define ROCK_NATTEMPTS 3   // total grip attempts, including the first

// TODO: tune every angle per joint, per rock, once you're testing on the
// real field layout. These are placeholders and are almost certainly wrong.
//
// One slot per physical rock (6), indexed by stop order along the track -
// see rockIndex in StateMachine.cpp, which counts every stop (metal or not).

struct RockWaypoints
{
    ArmPose orient; // face the rock, claw open
    ArmPose grab;   // reach in, close claw
};

static const RockWaypoints ROCK_POSITIONS[6] = {
    /* stop 1 */ {{0, 85, 205, 0, false}, {0, 85, 205, 0, true}},
    /* stop 2 */ {{0, 85, 205, 0, false}, {0, 85, 205, 0, true}},
    /* stop 3 */ {{0, 85, 205, 0, false}, {0, 85, 205, 0, true}},
    /* stop 4 */ {{0, 85, 205, 0, false}, {0, 85, 205, 0, true}},
    /* stop 5 */ {{0, 85, 205, 0, false}, {0, 85, 205, 0, true}},
    /* stop 6 */ {{0, 85, 205, 0, false}, {0, 85, 205, 0, true}},
};

// Shared for every rock - once it's out of the ground, lifting and
// swinging back over the chassis doesn't depend on which rock it was.
static const ArmPose RETRACT_ROCK  = {0, 160, 205, 0, true};
static const ArmPose RECENTER_ROCK = {90, 160, 205, 0, true};

// Reach in and close the claw for a specific rock. Caller checks the grip
// result before deciding whether to retract (success) or retry/abort.
inline void rockReachSequence(TaskManager& taskManager, int rockIndex)
{
    const RockWaypoints& wp = ROCK_POSITIONS[rockIndex];
    taskManager.executeMove(wp.orient);
    taskManager.executeMove(wp.grab);
}

// Lift out and swing back over the robot body. Only call this once the
// grip has been confirmed.
inline void rockRetractSequence(TaskManager& taskManager)
{
    taskManager.executeMove(RETRACT_ROCK);
    taskManager.executeMove(RECENTER_ROCK);
}

#endif // ROCK_H