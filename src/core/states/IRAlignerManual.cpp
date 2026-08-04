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

// Speed used while driving toward the target position.
static constexpr int TRAVEL_SPEED = 80;

// Maximum allowed final position error, checked per-axis (see update()).
// Pose data (UART::PoseData) is in millimeters -- see
// Sensor_ESP_Arduino/src/main.cpp -- so this must be in mm too.
static constexpr float ARRIVAL_TOLERANCE_MM = 50.0f;

// FIX: added -- driveToSolarPanelCoordinates() used to call blocking
// drive.driveBackward()/strafeRightWithDist() helpers with no way for this
// module to bail out except through their own (buggy) invalid-reading counter
static constexpr uint8_t MAX_INVALID_READINGS = 10;
static constexpr unsigned long PHASE_TIMEOUT_MS = 4000;

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

// FIX: added -- needed so DRIVING can be advanced a tick at a time in update()
// instead of blocking start() until the whole maneuver finishes
enum class DrivePhase
{
    Y,
    X
};

static ManualAlignState currentState = ManualAlignState::IDLE;
static DrivePhase currentPhase = DrivePhase::Y;

static float targetX = 0.0f;
static float targetY = 0.0f;

static unsigned long phaseStartTime = 0;
static uint8_t invalidReadingCount = 0;

// --------------------------------------------------
// Internal helpers
// --------------------------------------------------

static void driveToSolarPanelCoordinates()
{
    // FIX: this used to take currentX/currentY and compute
    // SOLAR_PANEL_FROM_SIDE_TAPES_DX/DY - currentX as if the constant were an
    // absolute coordinate. That's only correct if world (0,0) happens to be
    // exactly where the side-tape crossing occurred. Anchoring to
    // SlowTapeFollowing's recorded crossing position instead makes this
    // correct regardless of where in the run that crossing happened.
    targetX = SlowTapeFollowing::getSideTapeX() + SOLAR_PANEL_FROM_SIDE_TAPES_DX;
    targetY = SlowTapeFollowing::getSideTapeY() + SOLAR_PANEL_FROM_SIDE_TAPES_DY;
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

    // FIX: this used to call driveToSolarPanelCoordinates(startLoc.x, startLoc.y)
    // and block here until the entire two-step drive finished (that function called
    // drive.driveBackward()/strafeRightWithDist(), both blocking while() loops) --
    // stalling the whole board's main loop for the whole maneuver. Now start() just
    // sets the target and phase; update() below drives it one tick at a time.
    driveToSolarPanelCoordinates();
    currentPhase = DrivePhase::Y;
    phaseStartTime = millis();
    invalidReadingCount = 0;

    currentState = ManualAlignState::DRIVING;
}

void update()
{
    // FIX: this used to be empty because start() did the whole drive itself.
    // Now this is where the actual driving happens, a tick at a time.
    if (currentState != ManualAlignState::DRIVING)
    {
        return;
    }

    const UART::PoseData& pose = UART::getPoseData();

    if (!pose.valid)
    {
        invalidReadingCount++;
    }
    else
    {
        invalidReadingCount = 0;

        if (currentPhase == DrivePhase::Y)
        {
            //step 1: drive toward target Y --> through testing the position sensor we determined that fwd/bckwd changes Y.
            if (fabsf(pose.y - targetY) <= ARRIVAL_TOLERANCE_MM)
            {
                currentPhase = DrivePhase::X;
                phaseStartTime = millis();
            }
            else if (pose.y < targetY)
            {
                drive.backward(TRAVEL_SPEED);
            }
            else
            {
                drive.forward(TRAVEL_SPEED); // FIX: original always called driveBackward() even when the target actually needed forward motion
            }
        }
        else
        {
            //step 2: strafe until align in X
            if (fabsf(pose.x - targetX) <= ARRIVAL_TOLERANCE_MM)
            {
                // FIX: check BOTH axes independently before declaring success, not just X --
                // |finalY - targetY| could theoretically have drifted back out while strafing
                if (fabsf(pose.y - targetY) <= ARRIVAL_TOLERANCE_MM)
                {
                    drive.stop();
                    currentState = ManualAlignState::FINISHED;
                }
                else
                {
                    drive.stop();
                    currentState = ManualAlignState::FAILED;
                }
                return;
            }
            else if (pose.x < targetX)
            {
                drive.strafeRight(TRAVEL_SPEED);
            }
            else
            {
                drive.strafeLeft(TRAVEL_SPEED); // FIX: original always called strafeRightWithDist() even when the target needed strafeLeft instead
            }
        }
    }

    //step 3 (after testing:) hardcode rotation to align properly if needed
    /*
    float startTime = millis();
    while (millis() - startTime <= ROTATE_TIME_MS) {
        drive.rotateAboutCenter(ROTATE_SPEED);
    };
    */

    // FIX: added -- neither invalid-reading nor stuck-forever protection existed once
    // start() stopped blocking, since the old bail-out lived inside MecanumDrive's helpers
    if (invalidReadingCount >= MAX_INVALID_READINGS || millis() - phaseStartTime >= PHASE_TIMEOUT_MS)
    {
        drive.stop();
        currentState = ManualAlignState::FAILED;
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

// used by the website debug box -- phase- and direction-aware message,
// e.g. "Driving backward to checkpoint (Y)" / "Strafing right to panel (X)"
const char* getDebugStatus()
{
    if (currentState != ManualAlignState::DRIVING)
    {
        return getStateName();
    }

    const UART::PoseData& pose = UART::getPoseData();

    if (currentPhase == DrivePhase::Y)
    {
        return (pose.y < targetY)
            ? "Driving backward to checkpoint (Y)"
            : "Driving forward to checkpoint (Y)";
    }

    return (pose.x < targetX)
        ? "Strafing right to panel (X)"
        : "Strafing left to panel (X)";
}

} // namespace IRAlignerManual