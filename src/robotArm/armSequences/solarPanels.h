#ifndef SOLAR_PANELS_H
#define SOLAR_PANELS_H

#include "../ArmController2.h"
#include "../taskManager.h"

#define CLAW_CLOSED_PANEL 43

//positions to reach the solar panels
static const ArmPose REACH_PANEL = {40, 165, 270, 190, 35}; 
static const ArmPose GRAB_PANEL = {40, 165, 270, 190, CLAW_CLOSED_PANEL}; //update after testing
static const ArmPose RECENTER = {135, 90, 270, 190, CLAW_CLOSED_PANEL};

inline void solarPanelSequence(TaskManager& taskManager) {

    taskManager.executeMove(REACH_PANEL); 
    delay(250); 
    taskManager.executeMove(GRAB_PANEL); 
    delay(400); 
    taskManager.executeMove(RECENTER); 

}

#endif // SOLAR_PANELS_H