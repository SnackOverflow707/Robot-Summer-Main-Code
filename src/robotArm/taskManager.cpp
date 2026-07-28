#include "taskManager.h" 
#include <cstring>

//hey i did almost all of this by myself! 
const char* PICKUP_JOINT_ORDER[5] = {"shoulder", "elbow", "base", "wrist", "claw"}; 
// use initializer list to bind the reference variable
TaskManager::TaskManager(ArmController2& armRef) : _arm(armRef) {
    // Constructor body can stay empty
}


void TaskManager::executeMove(
    const ArmPose& waypoint,
    const char* jointOrder[]
)
{
    for (int word = 0; word < 5; ++word)
    {
        if (strcmp(jointOrder[word], "base") == 0)
        {
            _arm.setBase(waypoint.baseAngle);
        }
        else if (strcmp(jointOrder[word], "shoulder") == 0)
        {
            _arm.setShoulder(waypoint.shoulderAngle);
        }
        else if (strcmp(jointOrder[word], "elbow") == 0)
        {
            _arm.setElbow(waypoint.elbowAngle);
        }
        else if (strcmp(jointOrder[word], "wrist") == 0)
        {
            _arm.setWrist(waypoint.wristAngle);
        }
        else if (strcmp(jointOrder[word], "claw") == 0)
        {
            _arm.setClaw(waypoint.clawAngle);
        }
        else
        {
            printf("Joint name not recognized: %s\n", jointOrder[word]);
        }
    }
}


void TaskManager::executeMove(
    const ArmPose& waypoint
)
{
    const char* jointOrder[] = {
        "base",
        "shoulder",
        "elbow",
        "wrist",
        "claw"
    };

    executeMove(waypoint, jointOrder);
}



bool TaskManager::checkTime(unsigned long startTime, unsigned long max_time) {
    unsigned long currentTime = millis();
    return (currentTime - startTime) <= max_time;
}

bool TaskManager::objectGripCheckSequence(const ArmPose& objectLoc, int nAttempts) {
    delay(DELAY);

    // Didn't reach full closure -> object is present
    if (!_arm.clawSwitch.fullyClosed(GRIP_CHECK_DURATION)) {
        return true;
    }

    ArmPose backupLoc = objectLoc;
    backupLoc.elbowAngle += BACKUP;

    ArmPose reEntryLoc = objectLoc;
    reEntryLoc.clawAngle = CLAW_OPEN;

    for (int re_attempt = 1; re_attempt < nAttempts; re_attempt++) {

        _arm.openClaw();
        delay(DELAY);

        executeMove(backupLoc);
        delay(DELAY);

        executeMove(reEntryLoc);
        delay(DELAY);

        _arm.closeClaw();
        delay(DELAY);

        if (!_arm.clawSwitch.fullyClosed(GRIP_CHECK_DURATION)) {
            return true;
        }
    }

return false;
}




//run this function when you arrive at object location and close the claw. 
//Assumes arm is at the desired object location and the claw is closed. 
/*function: checks if the object is gripped. if it's gripped, returns true. If not gripped on the first attempt, 
the arm moves up, returns to the original object location, and tries again. If it continues to fail, it backs up and 
returns false.
Based on the definition where "gripped" = microswitch is PRESSED.*/
/*
bool TaskManager::objectGripCheckSequence(const Waypoint& objectLoc, int nAttempts) {

    delay(DELAY); 

    if (_arm.clawSwitch.objectGripped(GRIP_CHECK_DURATION)) {
        return true; 
    }

    Waypoint backupLoc = objectLoc;
    backupLoc.elbowAngle += BACKUP; 

    Waypoint reEntryLoc = objectLoc; 
    reEntryLoc.clawClosed = false; //because the original object loc has the claw closed. 

    for (int re_attempt = 1; re_attempt < nAttempts; re_attempt++) {

        _arm.openClaw(); 
        delay(DELAY);
        executeMove(backupLoc);
        delay(DELAY);
        executeMove(reEntryLoc); 
        delay(DELAY);
        _arm.closeClaw(); 
        delay(DELAY);

        if (_arm.clawSwitch.objectGripped(GRIP_CHECK_DURATION)) {
            return true; 
        }
    }
    return false; 
}
    */

//Use this for a series of motions that don't involve the claw closing around an object 
void TaskManager::executeSequence(const std::vector<ArmPose>& waypoints) {
    for (const auto& waypoint : waypoints) {
        executeMove(waypoint); 
    }
}

