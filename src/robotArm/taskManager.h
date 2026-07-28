#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#pragma once
#include "ArmController2.h"
#include <vector>
#include "helperMaterials.h"

#define BACKUP 30 //degrees to move the elbow upwards 
#define DELAY 1000 //ms, for the object detection sequence 
#define GRIP_CHECK_DURATION 1000 //ms 

const char* PICKUP_JOINT_ORDER[] = {"shoulder", "elbow", "base", "wrist", "claw"}; 

class TaskManager {
private:
    ArmController2& _arm; 

public: 

    TaskManager(ArmController2& armRef); 
    void executeMove(const ArmPose& waypoint,
                 const char* jointOrder[] = {"base", "shoulder", "elbow", "wrist", "claw"}); 
    void executeSequence(const std::vector<ArmPose>& waypoints); 
    bool objectGripCheckSequence(const ArmPose& objectLoc, int nAttempts);
    bool checkTime(unsigned long startTime, unsigned long max_time); 

}; 

#endif // TASK_MANAGER_H 
