#ifndef ROCK_H
#define ROCK_H

#include "../ArmController2.h"
#include "../taskManager.h"

#define CLAW_CLOSED_ROCK 20 //update 


/*I'm rewriting this based on the fact that the chassis should stop with the rock at the same location relative 
to the arm base each time because we have the positional sensors.*/

const char* rockPositions[] = {
    "right", "left", "right", "right", "left", "right"
}; 

const char* rockPickupOrder[] = {
    "base", "claw", "elbow", "shoulder", "wrist" 
};

static const ArmPose NEUTRAL = {}; 
static const ArmPose ROCK_TO_CHASSIS = {}; //brings the rock to the rock mount on the chassis
static const ArmPose PLACE_ROCK = {}; //places the rock on the mount. 

static const ArmPose REACH_LEFT = {}; 
static const ArmPose REACH_RIGHT = {}; 
static const ArmPose GRAB_LEFT = {}; 
static const ArmPose GRAB_RIGHT = {}; 

struct rockPoses {
    ArmPose reach; 
    ArmPose grab; 
}; 

rockPoses rightRockPoses = {REACH_RIGHT, GRAB_RIGHT}; 
rockPoses leftRockPoses = {REACH_LEFT, GRAB_LEFT}; 

static const std::vector<ArmPose> PLACE_ROCK_ON_CHASSIS = {
    ROCK_TO_CHASSIS, 
    PLACE_ROCK, 
    NEUTRAL 
}; 

inline void rockReachSequence(TaskManager& taskManager, int rockIndex)
{
    rockPoses rp; 
    if (rockPositions[rockIndex] == "right") {
        rp = rightRockPoses; 
    } 
    else {
        rp = leftRockPoses; 
    }

    taskManager.executeMove(rp.reach, rockPickupOrder); 
    delay(250); //so position can be confirmed before closing 
    taskManager.executeMove(rp.grab); 
    delay(400); //so claw can stabilize around rock 

}

inline void rockGrabSequence(TaskManager& taskManager) {
    taskManager.executeSequence(PLACE_ROCK_ON_CHASSIS); 
}







//---------------------- Ken's code -----------------------//


/*
#define ROCK_NATTEMPTS 3   // total grip attempts, including the first

struct RockWaypoints
{
    ArmPose orient; // face the rock, claw open
    ArmPose grab;   // reach in, close claw
};

static const RockWaypoints ROCK_POSITIONS[6] = {
     {{0, 85, 205, 0, CLAW_OPEN}, {0, 85, 205, 0, CLAW_CLOSED_ROCK }},
    {{0, 85, 205, 0, CLAW_OPEN}, {0, 85, 205, 0, CLAW_CLOSED_ROCK }},
   {{0, 85, 205, 0, CLAW_OPEN}, {0, 85, 205, 0, CLAW_CLOSED_ROCK }},
    {{0, 85, 205, 0, CLAW_OPEN}, {0, 85, 205, 0, CLAW_CLOSED_ROCK }},
    {{0, 85, 205, 0, CLAW_OPEN}, {0, 85, 205, 0, CLAW_CLOSED_ROCK }},
     {{0, 85, 205, 0, CLAW_OPEN}, {0, 85, 205, 0, CLAW_CLOSED_ROCK }},
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
*/ 

#endif // ROCK_H