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

// --------------------------------------------------
// Configuration
// --------------------------------------------------

static constexpr int TRAVEL_SPEED = 80;
static constexpr float ARRIVAL_TOLERANCE_M = 0.05f;

static constexpr uint8_t MAX_INVALID_READINGS = 10;

// Per-phase wall-clock safety net, in case pose feedback never
// converges (e.g. a stalled wheel) even though data keeps reading valid.
static constexpr unsigned long PHASE_TIMEOUT_MS = 4000;

// UPDATE AFTER TESTING
static constexpr int ROTATE_SPEED = 140;
static constexpr unsigned long ROTATE_TIME_MS = 3500;

enum class ManualAlignState
{
    IDLE,
    DRIVING_Y,
    DRIVING_X,
    FINISHED,
    FAILED
};

static ManualAlignState currentState = ManualAlignState::IDLE;
static unsigned long stateStartTime = 0;

// Absolute world-frame target, computed once in start() as the recorded
// side-tape-crossing position (from SlowTapeFollowing) plus the
// calibrated panel offset. FIX: previously this code treated
// SOLAR_PANEL_FROM_SIDE_TAPES_DX/DY as if they WERE the absolute target
// coordinates directly, ignoring where the side-tape crossing actually
// happened -- which only works if world (0,0) always exactly coincides
// with the side-tape crossing. Anchoring to SlowTapeFollowing's recorded
// crossing position makes this correct regardless of where in the run
// that crossing occurred.
static float targetX = 0.0f;
static float targetY = 0.0f;

static uint8_t invalidReadingCount = 0;


static void changeState(ManualAlignState newState)
{
    drive.stop();
    currentState = newState;
    stateStartTime = millis();
    invalidReadingCount = 0;
}

static bool hasArrived()
{
    const UART::PoseData& pose = UART::getPoseData();

    if (!pose.valid)
    {
        return false;
    }

    const float dx = fabsf(pose.x - targetX);
    const float dy = fabsf(pose.y - targetY);

    return (dx <= ARRIVAL_TOLERANCE_M) && (dy <= ARRIVAL_TOLERANCE_M);
}


void begin()
{
    drive.stop();
    currentState = ManualAlignState::IDLE;
}

void start()
{
    if (currentState == ManualAlignState::DRIVING_Y ||
        currentState == ManualAlignState::DRIVING_X)
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

    targetX = SlowTapeFollowing::getSideTapeX() + SOLAR_PANEL_FROM_SIDE_TAPES_DX;
    targetY = SlowTapeFollowing::getSideTapeY() + SOLAR_PANEL_FROM_SIDE_TAPES_DY;

    changeState(ManualAlignState::DRIVING_Y);
}

void update()
{
    switch (currentState)
    {
        case ManualAlignState::IDLE:
        case ManualAlignState::FINISHED:
        case ManualAlignState::FAILED:
        {
            break;
        }

        // fwd/bwd changes Y (per sensor testing).
        case ManualAlignState::DRIVING_Y:
        {
            const UART::PoseData& pose = UART::getPoseData();

            if (!pose.valid)
            {
                ++invalidReadingCount;
            }
            else
            {
                invalidReadingCount = 0;

                if (fabsf(pose.y - targetY) <= ARRIVAL_TOLERANCE_M)
                {
                    changeState(ManualAlignState::DRIVING_X);
                    break;
                }

                // FIX: original code always called driveBackward()
                // unconditionally, which silently did nothing if the
                // target actually required driving forward instead
                // (negative delta). Drive whichever direction closes
                // the gap.
                if (pose.y < targetY)
                {
                    drive.backward(TRAVEL_SPEED);
                }
                else
                {
                    drive.forward(TRAVEL_SPEED);
                }
            }

            if (invalidReadingCount >= MAX_INVALID_READINGS ||
                millis() - stateStartTime >= PHASE_TIMEOUT_MS)
            {
                changeState(ManualAlignState::FAILED);
            }

            break;
        }

        // strafe changes X (per sensor testing).
        case ManualAlignState::DRIVING_X:
        {
            const UART::PoseData& pose = UART::getPoseData();

            if (!pose.valid)
            {
                ++invalidReadingCount;
            }
            else
            {
                invalidReadingCount = 0;

                if (fabsf(pose.x - targetX) <= ARRIVAL_TOLERANCE_M)
                {
                    changeState(hasArrived()
                        ? ManualAlignState::FINISHED
                        : ManualAlignState::FAILED);
                    break;
                }

                if (pose.x < targetX)
                {
                    drive.strafeRight(TRAVEL_SPEED);
                }
                else
                {
                    drive.strafeLeft(TRAVEL_SPEED);
                }
            }

            if (invalidReadingCount >= MAX_INVALID_READINGS ||
                millis() - stateStartTime >= PHASE_TIMEOUT_MS)
            {
                changeState(ManualAlignState::FAILED);
            }

            break;
        }
    }

    // TODO (after testing): optional hardcoded rotation step to square
    // up with the panel, following the same non-blocking pattern -- add
    // a ROTATING state here rather than a blocking while() loop like the
    // commented-out original.
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
        case ManualAlignState::IDLE:      return "Idle";
        case ManualAlignState::DRIVING_Y: return "Driving to Solar Panel (Y)";
        case ManualAlignState::DRIVING_X: return "Driving to Solar Panel (X)";
        case ManualAlignState::FINISHED:  return "Manual Alignment Finished";
        case ManualAlignState::FAILED:    return "Manual Alignment Failed";
    }

    return "Unknown";
}

} // namespace IRAlignerManual
