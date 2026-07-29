#ifndef TOWER_H
#define TOWER_H

#include "../ArmController2.h"
#include "../taskManager.h"

#define TOWERS_TO_ATTEMPT 3
#define TOWER_CLAW_CLOSED 43

//tower positions 
static const ArmPose REACH_TOWER_1 = {155, 170, 235, 70, 20}; 
static const ArmPose GRAB_TOWER_1 = {155, 170, 235, 70, TOWER_CLAW_CLOSED}; 
static const ArmPose REACH_TOWER_2 = {160, 175, 220, 65, 0}; 
static const ArmPose GRAB_TOWER_2 = {160, 175, 220, 65, TOWER_CLAW_CLOSED}; 
static const ArmPose REACH_TOWER_3 = {175, 195, 170, 25, 0}; 
static const ArmPose GRAB_TOWER_3 = {175, 195, 170, 25, TOWER_CLAW_CLOSED}; 


const char* pickupOrder[] = {
    "shoulder",
    "base",
    "wrist",
    "elbow",
    "claw"
};
//all tower positions 
//repeat sequence
static const std::vector<ArmPose> ALL_TOWERS = {
    REACH_TOWER_1, 
    GRAB_TOWER_1, 
    REACH_TOWER_2, 
    GRAB_TOWER_2, 
    REACH_TOWER_3, 
    GRAB_TOWER_3, 
};

//repeat positions 
static const ArmPose ORIENT = {HOME_BASE, HOME_SHOULDER, HOME_ELBOW, 90, HOME_CLAW}; //base turns 90deg, claw is open
static const ArmPose RETRACT = {160, 120, 180, 60, 43}; 
static const ArmPose FUNNEL1 = {0, 95, 220, 70, 43}; 
static const ArmPose FUNNEL2 = {0, 95, 230, 70, 43}; //version 2: 0, 85, 235, 60, open/closed
static const ArmPose DROP_TOWER = {0, 95, 230, 70, 0};  

//repeat sequence
static const std::vector<ArmPose> TOWER_DROP_IN_FUNNEL = {
    RETRACT,
    FUNNEL1,
    FUNNEL2,
    DROP_TOWER,
    ORIENT
};

void towerSequence(TaskManager& taskManager) {

    taskManager.executeMove(ORIENT); 

    for (int step = 0; step < 2*TOWERS_TO_ATTEMPT; step+=2){
        taskManager.executeMove(ALL_TOWERS[step], pickupOrder); 
        taskManager.executeMove(ALL_TOWERS[step+1]); 
        taskManager.executeSequence(TOWER_DROP_IN_FUNNEL); 
    }

    
}




#endif // TOWER_H