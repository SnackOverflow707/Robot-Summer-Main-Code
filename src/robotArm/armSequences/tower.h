#ifndef TOWER_H
#define TOWER_H

#include "../ArmController2.h"
#include "../taskManager.h"

#define TOWERS_TO_ATTEMPT 3
#define TOWER_CLAW_CLOSED 43

//tower positions 
static const ArmPose PRE_TOWER_1 = {135, 140, 225, 50, 35}; 
static const ArmPose REACH_TOWER_1 = {155, 178, 225, 50, 35}; 
static const ArmPose GRAB_TOWER_1 = {155, 178, 225, 50, TOWER_CLAW_CLOSED}; 
static const ArmPose LIFT_TOWER_1 = {155, 180, 183, 15, TOWER_CLAW_CLOSED}; 
static const ArmPose REACH_TOWER_2 = {161, 194, 184, 13, 35}; 
static const ArmPose GRAB_TOWER_2 = {161, 194, 184, 13, TOWER_CLAW_CLOSED}; 
static const ArmPose REACH_TOWER_3 = {176, 205, 158, 13, 35}; 
static const ArmPose GRAB_TOWER_3 = {176, 205, 158, 13, TOWER_CLAW_CLOSED}; 


const char* moveToPieceOrder[] = {
    "base", 
    "claw",
    "wrist", 
    "elbow", 
    "shoulder"
};

const char* pickupOrder[] = {
    "claw", 
    "shoulder",
    "base", 
    "elbow", 
    "wrist"
};

const char* funnelOrder[] = {
    "shoulder", 
    "wrist", 
    "elbow", 
    "base", 
    "claw"
};

const char* liftTower1Order[] {
    "elbow", "wrist", "base", "shoulder", "claw"
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
//static const ArmPose ORIENT = {HOME_BASE, HOME_SHOULDER, HOME_ELBOW, 90, HOME_CLAW};
//static const ArmPose RETRACT = {160, 120, 180, 60, TOWER_CLAW_CLOSED}; 
static const ArmPose FUNNEL1 = {0, 95, 210, 70, TOWER_CLAW_CLOSED}; 
static const ArmPose FUNNEL2 = {0, 95, 230, 70, TOWER_CLAW_CLOSED}; //version 2: 0, 85, 235, 60, open/closed
static const ArmPose DROP_TOWER = {0, 95, 230, 70, CLAW_OPEN};  

//repeat sequence
static const std::vector<ArmPose> TOWER_DROP_IN_FUNNEL = {
    FUNNEL1,
    FUNNEL2,
    DROP_TOWER,

};

void towerSequence(TaskManager& taskManager) {

    //taskManager.executeMove(ORIENT); 

    for (int step = 0; step < 2*TOWERS_TO_ATTEMPT; step+=2) {
        if (step == 0) {
            taskManager.executeMove(PRE_TOWER_1, pickupOrder); //Stops gettiung caught on the gimble 
        }
        taskManager.executeMove(ALL_TOWERS[step], moveToPieceOrder); //reach the tower
        delay(250); 
        taskManager.executeMove(ALL_TOWERS[step+1], pickupOrder); //grab the tower
        delay(500); 

        if (step == 0) {
            taskManager.executeMove(LIFT_TOWER_1, liftTower1Order); //first tower needs extra space from metal detector. 
        }

        taskManager.executeMove(FUNNEL1, funnelOrder);
        delay(100); 
        taskManager.executeMove(FUNNEL2, funnelOrder);
        delay(250); 
        taskManager.executeMove(DROP_TOWER); 
        delay(100); 

    }

    
}




#endif // TOWER_H