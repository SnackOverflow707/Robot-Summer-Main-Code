#ifndef TOWER_H
#define TOWER_H

#include "../ArmController2.h"
#include "../taskManager.h"

constexpr unsigned long MAX_TOWER_TIME = 25UL * 1000UL; //maximum amount of time to attempt the towers, in ms
#define TOWERS_TO_ATTEMPT 1 

//tower positions 
static const ArmPose REACH_TOWER_1 = {}; 
static const ArmPose GRAB_TOWER_1 = {}; 
static const ArmPose REACH_TOWER_2 = {}; 
static const ArmPose GRAB_TOWER_2 = {}; 
static const ArmPose REACH_TOWER_3 = {}; 
static const ArmPose GRAB_TOWER_3 = {}; 

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
static const ArmPose ORIENT = {90, HOME_SHOULDER, HOME_ELBOW, HOME_WRIST, false}; //base turns 90deg, claw is open
static const ArmPose RETRACT = {}; 
static const ArmPose FUNNEL = {}; 
static const ArmPose DROP_TOWER = {}; 

//repeat sequence
static const std::vector<ArmPose> TOWER_DROP_IN_FUNNEL = {
    RETRACT,
    FUNNEL,
    DROP_TOWER,
    ORIENT
};

void towerSequence(TaskManager& taskManager) {

    taskManager.executeMove(ORIENT); 
    int startTime = millis(); 

    for (int step = 0; step < TOWERS_TO_ATTEMPT && taskManager.checkTime(startTime, MAX_TOWER_TIME); step+=2){
        taskManager.executeMove(ALL_TOWERS[step]); 
        taskManager.executeMove(ALL_TOWERS[step+1]); 
        taskManager.executeSequence(TOWER_DROP_IN_FUNNEL); 
    }
    
}




#endif // TOWER_H