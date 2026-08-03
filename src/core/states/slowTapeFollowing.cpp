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

static constexpr int TAPE_SPEED = 60;

static constexpr uint16_t MAG1_THRESHOLD = 20000;
static constexpr uint16_t MAG2_THRESHOLD = 3000;

static constexpr int SENSOR_SELECT_PIN = 11;

// give up if neither IR nor the panel-distance trigger fires in time -- FAILED used to be unreachable
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
    FINISHED,
    FAILED
};

static SlowTapeFollowState currentState = SlowTapeFollowState::IDLE;

static bool running = false;

static bool sideTapesPassed = false;
static bool sideTapeArmed = false;

static float currentX = 0.0f;
static float currentY = 0.0f;

static float sideTapeX = 0.0f;
static float sideTapeY = 0.0f;

static int sideTapeSightings = 0;

static unsigned long searchStartTime = 0; // needed for MAX_SEARCH_TIME_MS below



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

    const float travelledX =
        currentX - sideTapeX;

    const float travelledY =
        currentY - sideTapeY;

    const bool xReached =
        travelledX >=
        SOLAR_PANEL_FROM_SIDE_TAPES_DX -
        SEARCH_THRESHOLD_X;

    const bool yReached =
        travelledY >=
        SOLAR_PANEL_FROM_SIDE_TAPES_DY -
        SEARCH_THRESHOLD_Y;

    return xReached && yReached;
}



void begin()
{
    pinMode(
        SENSOR_SELECT_PIN,
        INPUT_PULLUP
    );

    running = false;

    currentX = 0.0f;
    currentY = 0.0f;

    sideTapeX = 0.0f;
    sideTapeY = 0.0f;

    sideTapeSightings = 0;
    sideTapesPassed = false;

    //Start unarmed so the robot must first see white before the first side tape detection can be counted.

    sideTapeArmed = false;

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
    searchStartTime = millis(); // needed for the timeout check in update()

    currentState = SlowTapeFollowState::FOLLOWING;
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

            //IR doesn't depend on the flow pose, so check it even if position data is temporarily invalid.
            if (isIRDetected(uartData.mag1, uartData.mag2)) {
                drive.stop(); // FIX: nothing stopped the motors here before -- tapeFollowStep() just stops being called, the last commanded speed keeps running
                currentState = SlowTapeFollowState::IR_DETECTED;
                break;
            }

            if (!flowData.valid)
            {
                // Keep tape following, but don't update pos until valid data is available
                break;
            }

            currentX = flowData.x;
            currentY = flowData.y;

            if (!sideTapesPassed && haveSideTapesPassed()) {
                sideTapeX = currentX;
                sideTapeY = currentY;
                sideTapesPassed = true;
            }
            else if (haveSolarPanelsPassed()) {
                drive.stop(); // FIX: same missing-stop issue as the IR_DETECTED branch above
                currentState = SlowTapeFollowState::DRIVE_TO_PANELS;
                break;
            }

            // FIX: added so this can't tape-follow forever if a side-tape crossing gets missed or the flow link dies
            if (millis() - searchStartTime >= MAX_SEARCH_TIME_MS) {
                drive.stop();
                currentState = SlowTapeFollowState::FAILED;
            }

            break;
        }

        case SlowTapeFollowState::IR_DETECTED:
        {
            // FIX: this used to jump straight to FINISHED, which erased the distinction between
            // IR_DETECTED and DRIVE_TO_PANELS one tick later -- the caller needs to see this state
            // to know it should launch the automatic IR aligner instead of the manual fallback
            break;
        }

        case SlowTapeFollowState::DRIVE_TO_PANELS:
        {
            // FIX: same problem as IR_DETECTED -- collapsing to FINISHED here meant the caller
            // could never tell it needed to launch the manual fallback instead
            break;
        }

        case SlowTapeFollowState::FINISHED:
        {
            // currently unreachable now that IR_DETECTED/DRIVE_TO_PANELS no longer auto-advance here
            break;
        }

        case SlowTapeFollowState::FAILED:
        {
            break;
        }
    }
}

void stop()
{
    running = false;
    drive.stop(); // FIX: added so calling stop() while still FOLLOWING actually halts the motors
    currentState = SlowTapeFollowState::IDLE;
}


bool haveSideTapesPassed()
{
    const SideSensorStatus sideStatus =
        getSideSensorStatus();

    // Seeing white arms the next tape crossing.
    if (!sideStatus.onTape)
    {
        sideTapeArmed = true; //the point of this variable is bc you have to leave the tape first in order to detect the second one.
        return false;
    }

    //count the no. of crossings.
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

bool isIRDetected(
    uint16_t mag1,
    uint16_t mag2
)
{
    const bool useMag1 =
        digitalRead(SENSOR_SELECT_PIN) == LOW;

    if (useMag1)
    {
        return mag1 > MAG1_THRESHOLD;
    }

    return mag2 > MAG2_THRESHOLD;
}

// FIX: added below -- without these, nothing outside this file could tell IR_DETECTED
// apart from DRIVE_TO_PANELS, which is the whole point of having two outcomes

bool wasIRDetected()
{
    return currentState == SlowTapeFollowState::IR_DETECTED;
}

bool needsManualFallback()
{
    return currentState == SlowTapeFollowState::DRIVE_TO_PANELS;
}

bool hasFailed()
{
    return currentState == SlowTapeFollowState::FAILED;
}

float getSideTapeX()
{
    return sideTapeX;
}

float getSideTapeY()
{
    return sideTapeY;
}
}