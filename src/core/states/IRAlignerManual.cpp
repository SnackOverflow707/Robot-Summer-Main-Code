#include "core/states/IRAlignerManual.h"
#include "core/states/SlowTapeFollowing.h"
#include "core/states/SolarPanelNavConstants.h"

#include <Arduino.h>
#include <math.h>

#include "actuators/MecanumDrive.h"
#include "comms/UART.h"

extern MecanumDrive drive;

namespace IRAlignerManual
{

// Speed used while driving toward the target Y position.
static constexpr int TRAVEL_SPEED = 100;

// Maximum allowed final position error on Y, checked against pose feedback.
// Pose data (UART::PoseData) is in millimeters (sensor_ESP_Arduino/src/main.cpp)
static constexpr float ARRIVAL_TOLERANCE_MM = 50.0f;

static constexpr uint8_t MAX_INVALID_READINGS = 10;
static constexpr unsigned long PHASE_TIMEOUT_MS = 10000; 

//static constexpr int HARDCODED_STRAFE_SPEED = 80;
//static constexpr unsigned long HARDCODED_STRAFE_TIME_MS = 2500;

//speed up attempt? Assume linear relationship 
static constexpr int HARDCODED_STRAFE_SPEED = 100;
static constexpr unsigned long HARDCODED_STRAFE_TIME_MS = 1650;


enum class ManualAlignState
{
    IDLE,
    STRAFE_TO_PANEL,
    FINISHED,
    FAILED
};

static ManualAlignState currentState = ManualAlignState::IDLE;

static float targetY = 0.0f;

static unsigned long phaseStartTime = 0;
static uint8_t invalidReadingCount = 0;

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
    if (currentState == ManualAlignState::STRAFE_TO_PANEL) {
        return;
    }

    const UART::PoseData startLoc = UART::getPoseData();
    if (!startLoc.valid) {
        drive.stop();
        currentState = ManualAlignState::FAILED;
        return;
    }

    // Anchored to SlowTapeFollowing's recorded crossing position, not
    // world (0,0) -- see SlowTapeFollowing::getSideTapeY().
    targetY = SlowTapeFollowing::getSideTapeY() + SOLAR_PANEL_CHECKPOINT_DY;

    phaseStartTime = millis();
    invalidReadingCount = 0;

    currentState = ManualAlignState::STRAFE_TO_PANEL;
}

void update()
{
    switch (currentState)
    {
            
        case ManualAlignState::STRAFE_TO_PANEL:
        {
            // Open-loop: no pose check, just run for a fixed time.
            if (millis() - phaseStartTime >= HARDCODED_STRAFE_TIME_MS)
            {
                drive.stop();
                currentState = ManualAlignState::FINISHED;
                break;
            }

            drive.strafeRight(HARDCODED_STRAFE_SPEED); 

            break;
        }

        case ManualAlignState::IDLE:
        case ManualAlignState::FINISHED:
        case ManualAlignState::FAILED:
            break;
    }
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
    return currentState == ManualAlignState::FINISHED;
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

        case ManualAlignState::STRAFE_TO_PANEL:
            return "Strafing to Solar Panel (X, hardcoded)";

        case ManualAlignState::FINISHED:
            return "Manual Alignment Finished";

        case ManualAlignState::FAILED:
            return "Manual Alignment Failed";
    }

    return "Unknown";
}

// used by the website debug box -- phase- and direction-aware message
const char* getDebugStatus()
{
    switch (currentState)
    {
        case ManualAlignState::STRAFE_TO_PANEL:
            return "Strafing right (hardcoded) to panel (X)";

        default:
            return getStateName();
    }
}

// used by the website's IR debug box
const char* getPhaseName()
{
    switch (currentState)
    {
        case ManualAlignState::IDLE:
            return "idle";

        case ManualAlignState::STRAFE_TO_PANEL:
            return "strafingToPanel";

        case ManualAlignState::FINISHED:
            return "finished";

        case ManualAlignState::FAILED:
            return "failed";
    }

    return "unknown";
}

} // namespace IRAlignerManual
