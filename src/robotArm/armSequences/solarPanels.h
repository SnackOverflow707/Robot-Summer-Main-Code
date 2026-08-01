#ifndef SOLAR_PANELS_H
#define SOLAR_PANELS_H

#include "../ArmController2.h"
#include "../taskManager.h"

#define CLAW_CLOSED_PANEL 43

//positions to reach the solar panels
static const ArmPose PRE_REACH_PANEL = {40, 165, 270, 190, 0}; 
static const ArmPose REACH_PANEL = {40, 165, 270, 190, 35}; 
static const ArmPose GRAB_PANEL = {40, 165, 270, 190, CLAW_CLOSED_PANEL}; //update after testing
static const ArmPose RECENTER = {135, 90, 270, 190, CLAW_CLOSED_PANEL};

const char* reachPanelOrder[] = { "claw", "shoulder", "elbow", "wrist", "base" };
const char* recenterOrder[] = {"elbow", "wrist", "base", "shoulder", "claw"};

inline void solarPanelSequence(TaskManager& taskManager) {
    taskManager.executeMove(PRE_REACH_PANEL, reachPanelOrder);
    taskManager.executeMove(REACH_PANEL, reachPanelOrder); 
    delay(250); 
    taskManager.executeMove(GRAB_PANEL); 
    delay(400); 
    taskManager.executeMove(RECENTER, recenterOrder); 

}

#endif // SOLAR_PANELS_H