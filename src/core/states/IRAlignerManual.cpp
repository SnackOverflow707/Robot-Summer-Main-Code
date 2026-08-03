
#include "core/states/IRAlignerManual.h"
#include "core/states/SlowTapeFollowing.h"

#include <Arduino.h>
#include <math.h>

#include "actuators/MecanumDrive.h"
#include "comms/UART.h"

extern MecanumDrive drive;

namespace IRAlignerManual
{

// Speed supplied to MecanumDrive::driveTo().
static constexpr int TRAVEL_SPEED = 80;

// Maximum allowed final position error.
static constexpr float ARRIVAL_TOLERANCE_M = 0.05f;

//UPDATE AFTER TESTING 
static constexpr int ROTATE_SPEED = 140;
static constexpr unsigned long ROTATE_TIME_MS = 3500;

enum class ManualAlignState
{
    IDLE,
    DRIVING,
    FINISHED,
    FAILED
};

static ManualAlignState currentState = ManualAlignState::IDLE;

// --------------------------------------------------
// Internal helpers
// --------------------------------------------------

static void driveToSolarPanelCoordinates(float currentX, float currentY)
{
    const float travelX = SOLAR_PANEL_FROM_SIDE_TAPES_DX - currentX;
    const float travelY = SOLAR_PANEL_FROM_SIDE_TAPES_DY - currentY;

    //step 1: drive backwards until align in Y --> through testing the position sensor we determined that fwd/bckwd changes Y. 
    drive.driveBackward(travelY, TRAVEL_SPEED); 

    //step 2: strafe until align in X 
    drive.strafeRightWithDist(travelX, TRAVEL_SPEED); 

    //step 3 (after testing:) hardcode rotation to align properly if needed 
    /* 
    float startTime = millis(); 
    while (millis() - startTime <= ROTATE_TIME_MS) {
        drive.rotateAboutCenter(ROTATE_SPEED); 
    }; 
    */

   drive.stop(); 
  
}


// --------------------------------------------------
// Public state controls
// --------------------------------------------------

void begin()
{
    drive.stop();
    currentState = ManualAlignState::IDLE;
}

void start()
{
    if (currentState == ManualAlignState::DRIVING)
    {
        return;
    }

    const UART::PoseData startLoc = UART::getPoseData();

    if (!startLoc.valid)
    {
        drive.stop();
        currentState = ManualAlignState::FAILED;
        return;
    }

    currentState = ManualAlignState::DRIVING;
    driveToSolarPanelCoordinates(startLoc.x,startLoc.y);
    //const UART::PoseData finalLoc = UART::getPoseData();
    currentState = ManualAlignState::FINISHED;

}

void update()
{
    //i don't think anything is needed here. 
}

void stop()
{
    drive.stop();
    currentState = ManualAlignState::IDLE;
}

// --------------------------------------------------
// Status
// --------------------------------------------------

bool isFinished()
{
    return currentState ==
        ManualAlignState::FINISHED;
}

bool hasFailed()
{
    return currentState == ManualAlignState::FAILED;
}

bool isDone()
{
    return isFinished() || hasFailed();
}

const char* getStateName()
{
    switch (currentState)
    {
        case ManualAlignState::IDLE:
            return "Idle";

        case ManualAlignState::DRIVING:
            return "Driving to Solar Panel";

        case ManualAlignState::FINISHED:
            return "Manual Alignment Finished";

        case ManualAlignState::FAILED:
            return "Manual Alignment Failed";
    }

    return "Unknown";
}

} // namespace IRAlignerManual

