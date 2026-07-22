#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#pragma once
#include "ArmController2.h"
#include <vector>
#include "helperMaterials.h"

#define BACKUP 30 //degrees to move the elbow upwards 
#define DELAY 1000 //ms, for the object detection sequence 
#define GRIP_CHECK_DURATION 1000 //ms 
#define NATTEMPTS 3 //total attempts including the initial attempt, if failed 

class TaskManager {
private:
    ArmController2& _arm; 

public: 
    TaskManager(ArmController2& armRef); 
    void executeMove(const ArmPose& waypoint); 
    void executeSequence(const std::vector<ArmPose>& waypoints); 
    bool objectGripCheckSequence(const ArmPose& objectLoc, int nAttempts);

}; 

#endif // TASK_MANAGER_H 
