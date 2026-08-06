#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#pragma once
#include "ArmController2.h"
#include <vector>
#include "helperMaterials.h"

#define BACKUP 30 //degrees to move the elbow upwards 
#define DELAY 1000 //ms, for the object detection sequence 
#define GRIP_CHECK_DURATION 1000 //ms 
extern const char* PICKUP_JOINT_ORDER[5];

class TaskManager {
private:
    ArmController2& _arm; 

public: 

    // Invoked after every individual joint move completes, so callers can
    // interleave other work (e.g. sensor checks) with an in-progress move
    // instead of waiting for the whole sequence to finish.
    using JointDoneCallback = void (*)();

    TaskManager(ArmController2& armRef);
    void executeMove(const ArmPose& waypoint, JointDoneCallback onJointDone = nullptr);

void executeMove(
    const ArmPose& waypoint,
    const char* jointOrder[],
    JointDoneCallback onJointDone = nullptr
);

    void executeSequence(const std::vector<ArmPose>& waypoints); 
    bool objectGripCheckSequence(const ArmPose& objectLoc, int nAttempts);
    bool checkTime(unsigned long startTime, unsigned long max_time); 

}; 

#endif // TASK_MANAGER_H 
