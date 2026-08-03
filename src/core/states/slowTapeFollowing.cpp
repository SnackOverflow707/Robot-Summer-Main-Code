#include "core/states/SlowTapeFollowing.h"
#include "core/states/SolarPanelNavConstants.h"

#include <Arduino.h>
#include <math.h>

#include "actuators/MecanumDrive.h"
#include "comms/UART.h"
#include "tape_logic/SideSensors.h"
#include "tape_logic/TapeFollower.h"

extern MecanumDrive drive;

namespace SlowTapeFollowing
{

// --------------------------------------------------
// Configuration
// --------------------------------------------------

static constexpr int TAPE_SPEED = 60;

static constexpr uint16_t MAG1_THRESHOLD = 20000;
static constexpr uint16_t MAG2_THRESHOLD = 3000;

static constexpr int SENSOR_SELECT_PIN = 11;

// Safety net: if neither IR nor the panel-distance trigger fires within
// this long, something is wrong (missed side-tape crossing, dead flow
// sensor, etc.) -- give up rather than tape-following forever. Tune
// against your actual course length / 2-minute heat budget.
static constexpr unsigned long MAX_SEARCH_TIME_MS = 15000;

// --------------------------------------------------
// Internal state
// --------------------------------------------------

enum class SlowTapeFollowState
{
    IDLE,
    FOLLOWING,
    DRIVE_TO_PANELS,
    IR_DETECTED,
    FINISHED,   // currently unreachable -- kept for API compatibility;
                // IR_DETECTED / DRIVE_TO_PANELS are now themselves the
                // terminal "success" states so the caller can tell them
                // apart (see note below).
    FAILED
};

static SlowTapeFollowState currentState = SlowTapeFollowState::IDLE;

static bool running = false;
static unsigned long stateStartTime = 0;

static bool sideTapesPassed = false;
static bool sideTapeArmed = false;

static float currentX = 0.0f;
static float currentY = 0.0f;

static float sideTapeX = 0.0f;
static float sideTapeY = 0.0f;

static int sideTapeSightings = 0;

// --------------------------------------------------
// Internal helpers
// --------------------------------------------------

static bool isFlowSensorDataValid()
{
    const UART::PoseData& flowData = UART::getPoseData();
    return flowData.valid;
}

static bool haveSolarPanelsPassed()
{
    if (!sideTapesPassed)
    {
        return false;
    }

    const float travelledX = currentX - sideTapeX;
    const float travelledY = currentY - sideTapeY;

    const bool xReached =
        travelledX >= SOLAR_PANEL_FROM_SIDE_TAPES_DX - SEARCH_THRESHOLD_X;

    const bool yReached =
        travelledY >= SOLAR_PANEL_FROM_SIDE_TAPES_DY - SEARCH_THRESHOLD_Y;

    return xReached && yReached;
}

static void changeState(SlowTapeFollowState newState)
{
    currentState = newState;
    stateStartTime = millis();
}

// --------------------------------------------------
// Public control
// --------------------------------------------------

void begin()
{
    pinMode(SENSOR_SELECT_PIN, INPUT_PULLUP);

    running = false;

    currentX = 0.0f;
    currentY = 0.0f;

    sideTapeX = 0.0f;
    sideTapeY = 0.0f;

    sideTapeSightings = 0;
    sideTapesPassed = false;

    // Start unarmed so the robot must first see white before the first
    // side tape detection can be counted.
    sideTapeArmed = false;

    currentState = SlowTapeFollowState::IDLE;
}

void start()
{
    currentX = 0.0f;
    currentY = 0.0f;

    sideTapeX = 0.0f;
    sideTapeY = 0.0f;

    sideTapeSightings = 0;
    sideTapesPassed = false;
    sideTapeArmed = false;

    running = true;

    changeState(SlowTapeFollowState::FOLLOWING);
}

void update()
{
    if (!running)
    {
        return;
    }

    switch (currentState)
    {
        case SlowTapeFollowState::IDLE:
        {
            break;
        }

        case SlowTapeFollowState::FOLLOWING:
        {
            tapeFollowStep();

            const UART::PoseData& flowData = UART::getPoseData();
            const UART::Data& uartData = UART::getData();

            // IR doesn't depend on the flow pose, so check it even if
            // position data is temporarily invalid.
            if (isIRDetected(uartData.mag1, uartData.mag2))
            {
                // FIX: stop the drive -- tapeFollowStep() was commanding
                // the motors every tick; nothing else will stop them once
                // we leave this state.
                drive.stop();
                changeState(SlowTapeFollowState::IR_DETECTED);
                break;
            }

            if (flowData.valid)
            {
                currentX = flowData.x;
                currentY = flowData.y;

                if (!sideTapesPassed && haveSideTapesPassed())
                {
                    sideTapeX = currentX;
                    sideTapeY = currentY;
                    sideTapesPassed = true;
                }
                else if (haveSolarPanelsPassed())
                {
                    drive.stop();
                    changeState(SlowTapeFollowState::DRIVE_TO_PANELS);
                    break;
                }
            }
            // else: keep tape following, but don't update position
            // until valid data is available again.

            // FIX: safety-net timeout -- previously nothing could ever
            // transition into FAILED, so a missed side-tape crossing or
            // dead flow link meant tape-following forever.
            if (millis() - stateStartTime >= MAX_SEARCH_TIME_MS)
            {
                drive.stop();
                changeState(SlowTapeFollowState::FAILED);
            }

            break;
        }

        // FIX: these no longer collapse into FINISHED one tick after
        // being entered. They ARE the terminal states now, so the outer
        // StateMachine has time to call getState() (or wasIRDetected() /
        // needsManualFallback() below) and decide whether to launch
        // IRAligner or IRAlignerManual, before this module is stopped.
        case SlowTapeFollowState::IR_DETECTED:
        case SlowTapeFollowState::DRIVE_TO_PANELS:
        case SlowTapeFollowState::FINISHED:
        case SlowTapeFollowState::FAILED:
        {
            break;
        }
    }
}

void stop()
{
    running = false;
    drive.stop();
    changeState(SlowTapeFollowState::IDLE);
}

bool haveSideTapesPassed()
{
    const SideSensorStatus sideStatus = getSideSensorStatus();

    // Seeing white arms the next tape crossing.
    if (!sideStatus.onTape)
    {
        sideTapeArmed = true; // must leave the tape before the next crossing counts
        return false;
    }

    if (sideStatus.onTape && sideTapeArmed)
    {
        sideTapeArmed = false;
        sideTapeSightings++;

        if (sideTapeSightings >= 2)
        {
            sideTapeSightings = 0;
            return true;
        }
    }

    return false;
}

bool isIRDetected(uint16_t mag1, uint16_t mag2)
{
    const bool useMag1 = digitalRead(SENSOR_SELECT_PIN) == LOW;

    if (useMag1)
    {
        return mag1 > MAG1_THRESHOLD;
    }

    return mag2 > MAG2_THRESHOLD;
}

// --------------------------------------------------
// Status
// --------------------------------------------------
//
// NOTE: none of this existed before. Without it, the outer StateMachine
// had no way to tell whether IR was actually detected vs. whether it
// should fall back to manual navigation -- which was the entire point
// of this module. Add matching declarations to SlowTapeFollowing.h.

SlowTapeFollowState getState()
{
    return currentState;
}

bool wasIRDetected()
{
    return currentState == SlowTapeFollowState::IR_DETECTED;
}

bool needsManualFallback()
{
    return currentState == SlowTapeFollowState::DRIVE_TO_PANELS;
}

bool isFinished()
{
    return wasIRDetected() || needsManualFallback();
}

bool hasFailed()
{
    return currentState == SlowTapeFollowState::FAILED;
}

bool isDone()
{
    return isFinished() || hasFailed();
}

float getSideTapeX()
{
    return sideTapeX;
}

float getSideTapeY()
{
    return sideTapeY;
}

const char* getStateName()
{
    switch (currentState)
    {
        case SlowTapeFollowState::IDLE:            return "Idle";
        case SlowTapeFollowState::FOLLOWING:       return "Slow Tape Following";
        case SlowTapeFollowState::DRIVE_TO_PANELS: return "Panel Range Reached (Manual Fallback)";
        case SlowTapeFollowState::IR_DETECTED:     return "IR Detected";
        case SlowTapeFollowState::FINISHED:        return "Finished";
        case SlowTapeFollowState::FAILED:          return "Failed";
    }

    return "Unknown";
}

} // namespace SlowTapeFollowing