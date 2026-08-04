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
static constexpr int TRAVEL_SPEED = 80;

// Maximum allowed final position error on Y, checked against pose feedback.
// Pose data (UART::PoseData) is in millimeters -- see
// Sensor_ESP_Arduino/src/main.cpp -- so this must be in mm too.
static constexpr float ARRIVAL_TOLERANCE_MM = 50.0f;

static constexpr uint8_t MAX_INVALID_READINGS = 10;
static constexpr unsigned long PHASE_TIMEOUT_MS = 10000; //chnging to smt ridiculous for now to debug why its not strafing 

// Open-loop strafe toward the panel once Y is aligned -- deliberately not
// pose-feedback-driven. This module's job is just to get roughly onto the
// panel's IR beacon; IRAligner (entered once this module finishes) does
// the accurate close-range positioning off the IR signal itself, which is
// more reliable at short range than the flow-sensor pose.
// TODO calibrate speed/time against SOLAR_PANEL_CHECKPOINT_DX via testing.
static constexpr int HARDCODED_STRAFE_SPEED = 80;
static constexpr unsigned long HARDCODED_STRAFE_TIME_MS = 1500;
//static constexpr bool STRAFE_RIGHT = SOLAR_PANEL_CHECKPOINT_DX >= 0.0f; //dx > 0 → target is to the right, dx < 0 → target is to the left

enum class ManualAlignState
{
    IDLE,
    DRIVING_Y,
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
    if (currentState == ManualAlignState::DRIVING_Y ||
        currentState == ManualAlignState::STRAFE_TO_PANEL)
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

    // Anchored to SlowTapeFollowing's recorded crossing position, not
    // world (0,0) -- see SlowTapeFollowing::getSideTapeY().
    targetY = SlowTapeFollowing::getSideTapeY() + SOLAR_PANEL_CHECKPOINT_DY;

    phaseStartTime = millis();
    invalidReadingCount = 0;

    currentState = ManualAlignState::DRIVING_Y;
}

void update()
{
    switch (currentState)
    {
        case ManualAlignState::DRIVING_Y:
        {
            drive.stop(); 
            currentState = ManualAlignState::STRAFE_TO_PANEL;
            phaseStartTime = millis();
            break;


            /*
            
            const UART::PoseData& pose = UART::getPoseData();
            if (!pose.valid)
            {
                ++invalidReadingCount;
            }
            else
            {
                invalidReadingCount = 0;

                if (fabsf(pose.y - targetY) <= ARRIVAL_TOLERANCE_MM)
                {
                    drive.stop();
                    currentState = ManualAlignState::STRAFE_TO_PANEL;
                    phaseStartTime = millis();
                    break;
                }
                else if (pose.y < targetY) 
                {
                    drive.backward(TRAVEL_SPEED);
                }
                else
                {
                    drive.forward(TRAVEL_SPEED);
                }
            }

            if (invalidReadingCount >= MAX_INVALID_READINGS ||
                millis() - phaseStartTime >= PHASE_TIMEOUT_MS)
            {
                drive.stop();
                currentState = ManualAlignState::FAILED;
            }

            break;*/
        }

        case ManualAlignState::STRAFE_TO_PANEL:
        {
            // Open-loop: no pose check, just run for a fixed time.
            if (millis() - phaseStartTime >= HARDCODED_STRAFE_TIME_MS)
            {
                drive.stop();
                currentState = ManualAlignState::FINISHED;
                break;
            }

            //if (STRAFE_RIGHT)
            //{
                drive.strafeRight(HARDCODED_STRAFE_SPEED);
            //}
            //else
            //{
                //drive.strafeLeft(HARDCODED_STRAFE_SPEED);
            //}

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

        case ManualAlignState::DRIVING_Y:
            return "Driving to Solar Panel (Y)";

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
        case ManualAlignState::DRIVING_Y:
        {
            const UART::PoseData& pose = UART::getPoseData();
            return (pose.y < targetY)
                ? "Driving backward to checkpoint (Y)"
                : "Driving forward to checkpoint (Y)";
        }

        case ManualAlignState::STRAFE_TO_PANEL:
            return "Strafing right (hardcoded) to panel (X)";

        default:
            return getStateName();
    }
}

// used by the website's IR debug box -- a short, unambiguous phase tag so
// the UI doesn't just say "Manual IR Aligning" (the top-level state name,
// which doesn't change between the Y and strafe sub-phases) the whole time.
const char* getPhaseName()
{
    switch (currentState)
    {
        case ManualAlignState::IDLE:
            return "idle";

        case ManualAlignState::DRIVING_Y:
            return "drivingY";

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
