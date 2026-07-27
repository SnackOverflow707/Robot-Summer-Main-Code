#ifndef SOLAR_PANELS_H
#define SOLAR_PANELS_H

#include "../ArmController2.h"
#include "../taskManager.h"

#define PANEL_NATTEMPTS 3 //total attempts including the initial attempt, if failed 

//positions to reach the solar panels
static const ArmPose ORIENT = { 0, 85, 205, 0, false}; //base turns 90deg, claw is open
static const ArmPose GRAB_PANEL = {0, 85, 205, 0, true}; //update after testing
static const ArmPose RETRACT = {0, 160, 205, 0, true}; //update after testing
static const ArmPose RECENTER = {90, 160, 205, 0,true};

inline void solarPanelSequence(TaskManager& taskManager) {

    taskManager.executeMove(ORIENT); 
    taskManager.executeMove(GRAB_PANEL); 
    //taskManager.objectGripCheckSequence(GRAB_PANEL, PANEL_NATTEMPTS);
    taskManager.executeMove(RETRACT);  
    taskManager.executeMove(RECENTER); 

}

#endif // SOLAR_PANELS_H