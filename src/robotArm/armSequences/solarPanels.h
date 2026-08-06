#ifndef SOLAR_PANELS_H
#define SOLAR_PANELS_H

#include "../ArmController2.h"
#include "../taskManager.h"

#define CLAW_CLOSED_PANEL 43

//positions to reach the solar panels
static const ArmPose REACH_PANEL_1k = {40, 155, 270, 165, 10}; 
static const ArmPose GRAB_PANEL_1k = {40, 155, 270, 165, CLAW_CLOSED_PANEL}; 
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

// --------------------------------------------------
// Fallback sequence
//
// Used when IRAlignerManual's strafe + IRAligner's close-range search
// both fail to find the IR beacon. The robot ends up positioned
// differently than in the normal aligned case, so this sequence needs
// its own set of angles -- fill these in after testing against the
// fallback position.
// --------------------------------------------------

// TODO: fill in with the angles measured for the fallback position.
//NOT TESTED YET. 
static const ArmPose REACH_PANEL_FALLBACK = {25, 155, 270, 165, 10};
static const ArmPose GRAB_PANEL_FALLBACK = {25, 155, 270, 165, CLAW_CLOSED_PANEL};
static const ArmPose RECENTER_FALLBACK = {135, 90, 270, 190, CLAW_CLOSED_PANEL};

inline void solarPanelFallbackSequence_1KHz(TaskManager& taskManager) {
    taskManager.executeMove(REACH_PANEL_FALLBACK, reachPanelOrder);
    delay(250);
    taskManager.executeMove(GRAB_PANEL_FALLBACK);
    delay(400);
    taskManager.executeMove(RECENTER_FALLBACK, recenterOrder);
}

#endif // SOLAR_PANELS_H