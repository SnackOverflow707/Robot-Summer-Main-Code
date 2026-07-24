#ifndef SOLAR_PANELS_H
#define SOLAR_PANELS_H

#include "../ArmController2.h"
#include "../taskManager.h"

#define PANEL_NATTEMPTS 3 //total attempts including the initial attempt, if failed 

//positions to reach the solar panels
static const ArmPose ORIENT = {90, HOME_SHOULDER, HOME_ELBOW, HOME_WRIST, false}; //base turns 90deg, claw is open
static const ArmPose GRAB_PANEL = {90, 90, 90, 90, true}; //update after testing
static const ArmPose RETRACT = {90, 120, 30, 110, true}; //update after testing
static const ArmPose RECENTER = {0, HOME_SHOULDER, HOME_ELBOW, HOME_WRIST, false};

void solarPanelSequence(TaskManager& taskManager) {

    taskManager.executeMove(ORIENT); 
    taskManager.executeMove(GRAB_PANEL); 
    //taskManager.objectGripCheckSequence(GRAB_PANEL, PANEL_NATTEMPTS);
    taskManager.executeMove(RETRACT);  
    taskManager.executeMove(RECENTER); 

}

#endif // SOLAR_PANELS_H