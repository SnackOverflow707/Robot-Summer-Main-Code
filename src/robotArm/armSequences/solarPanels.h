#ifndef SOLAR_PANELS_H
#define SOLAR_PANELS_H

#include "../ArmController2.h"
#include "../taskManager.h"

#define CLAW_CLOSED_PANEL 43

//positions to reach the solar panels
static const ArmPose REACH_PANEL_1k = {40, 155, 270, 165, 10}; 
static const ArmPose GRAB_PANEL_1k = {40, 155, 270, 165, CLAW_CLOSED_PANEL}; //update after testing
static const ArmPose RECENTER = {135, 90, 270, 190, CLAW_CLOSED_PANEL};

inline const char* reachPanelOrder[] = { "claw", "shoulder", "elbow", "wrist", "base" };
inline const char* recenterOrder[] = {"elbow", "wrist", "base", "shoulder", "claw"};

inline void solarPanelSequence_1KHz(TaskManager& taskManager) {
    taskManager.executeMove(REACH_PANEL_1k, reachPanelOrder); 
    delay(250); 
    taskManager.executeMove(GRAB_PANEL_1k); 
    delay(400); 
    taskManager.executeMove(RECENTER, recenterOrder); 
}

#endif // SOLAR_PANELS_H