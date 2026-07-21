#include "core/states/IRAligner.h"

#include <Arduino.h>

#include "actuators/MecanumDrive.h"
#include "comms/UART.h"

extern MecanumDrive drive;

namespace IRAligner
{

// --------------------------------------------------
// Configuration
// --------------------------------------------------

// Initial right strafe
static constexpr int STRAFE_SPEED = 100;
static constexpr unsigned long STRAFE_TIME_MS = 1900;

// Forward/backward IR search speed
static constexpr int SEARCH_SPEED = 60;

// Search backward for this amount of time
static constexpr unsigned long BACKWARD_TIME_MS = 2000;

// Then search forward for this amount of time
static constexpr unsigned long FORWARD_SEARCH_TIME_MS = 3000;

// Stop when the selected IR magnitude reaches this value
static constexpr uint16_t IR_FOUND_THRESHOLD = 100;

// true  = use mag2
// false = use mag1
static constexpr bool USE_MAG2 = true;

// --------------------------------------------------
// Internal states
// --------------------------------------------------

enum class AlignState
{
    IDLE,
    STRAFE_RIGHT,
    SEARCH_BACKWARD,
    SEARCH_FORWARD,
    FINISHED,
    NOT_FOUND
};

static AlignState currentState = AlignState::IDLE;
static unsigned long stateStartTime = 0;

// --------------------------------------------------
// Internal helpers
// --------------------------------------------------

static uint16_t getIRMagnitude()
{
    const UART::Data& data = UART::getData();

    if (!data.valid)
    {
        return 0;
    }

    return USE_MAG2 ? data.mag2 : data.mag1;
}

static bool targetWasFound()
{
    return getIRMagnitude() >= IR_FOUND_THRESHOLD;
}

static void changeState(AlignState newState)
{
    drive.stop();

    currentState = newState;
    stateStartTime = millis();

    switch (currentState)
    {
        case AlignState::IDLE:
            break;

        case AlignState::STRAFE_RIGHT:
            drive.strafeRight(STRAFE_SPEED);
            break;

        case AlignState::SEARCH_BACKWARD:
            drive.backward(SEARCH_SPEED);
            break;

        case AlignState::SEARCH_FORWARD:
            drive.forward(SEARCH_SPEED);
            break;

        case AlignState::FINISHED:
        case AlignState::NOT_FOUND:
            drive.stop();
            break;
    }
}

// --------------------------------------------------
// Public functions
// --------------------------------------------------

void begin()
{
    currentState = AlignState::IDLE;
    stateStartTime = 0;
    drive.stop();
}

void start()
{
    changeState(AlignState::STRAFE_RIGHT);
}

void update()
{
    // UART::update() must be called regularly elsewhere.

    switch (currentState)
    {
        case AlignState::IDLE:
            break;

        case AlignState::STRAFE_RIGHT:
            if (millis() - stateStartTime >= STRAFE_TIME_MS)
            {
                changeState(AlignState::SEARCH_BACKWARD);
            }
            break;

        case AlignState::SEARCH_BACKWARD:
            // Check for the target while moving backward.
            if (targetWasFound())
            {
                changeState(AlignState::FINISHED);
            }
            else if (millis() - stateStartTime >= BACKWARD_TIME_MS)
            {
                changeState(AlignState::SEARCH_FORWARD);
            }
            break;

        case AlignState::SEARCH_FORWARD:
            // Check for the target while moving forward.
            if (targetWasFound())
            {
                changeState(AlignState::FINISHED);
            }
            else if (
                millis() - stateStartTime >=
                FORWARD_SEARCH_TIME_MS
            )
            {
                changeState(AlignState::NOT_FOUND);
            }
            break;

        case AlignState::FINISHED:
        case AlignState::NOT_FOUND:
            drive.stop();
            break;
    }
}

void stop()
{
    changeState(AlignState::IDLE);
}

bool isFinished()
{
    return currentState == AlignState::FINISHED;
}

bool hasFailed()
{
    return currentState == AlignState::NOT_FOUND;
}

bool isDone()
{
    return isFinished() || hasFailed();
}

uint16_t getCurrentMagnitude()
{
    return getIRMagnitude();
}

const char* getStateName()
{
    switch (currentState)
    {
        case AlignState::IDLE:
            return "Idle";

        case AlignState::STRAFE_RIGHT:
            return "Strafe Right";

        case AlignState::SEARCH_BACKWARD:
            return "Search Backward";

        case AlignState::SEARCH_FORWARD:
            return "Search Forward";

        case AlignState::FINISHED:
            return "IR Found";

        case AlignState::NOT_FOUND:
            return "IR Not Found";
    }

    return "Unknown";
}

} // namespace IRAligner