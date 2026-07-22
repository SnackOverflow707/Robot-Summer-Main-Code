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
static constexpr int STRAFE_SPEED = 90;
static constexpr unsigned long STRAFE_TIME_MS = 1600;

// Search speeds
static constexpr int SEARCH_FAST_SPEED = 60;
static constexpr int SEARCH_SLOW_SPEED = 30;

// Search backward for this amount of time
static constexpr unsigned long BACKWARD_TIME_MS = 2000;

// Then search forward for this amount of time
static constexpr unsigned long FORWARD_SEARCH_TIME_MS = 3000;

// UART data must be newer than this
static constexpr unsigned long UART_TIMEOUT_MS = 250;

// Final detection thresholds
static constexpr uint16_t MAG1_FOUND_THRESHOLD = 20000;
static constexpr uint16_t MAG2_FOUND_THRESHOLD = 2000;

// Slow down when the signal reaches these values
static constexpr uint16_t MAG1_NEAR_THRESHOLD = 18000;
static constexpr uint16_t MAG2_NEAR_THRESHOLD = 1800;

// Number of consecutive readings needed to confirm detection
static constexpr uint8_t REQUIRED_FOUND_SAMPLES = 4;

// Exponential smoothing:
// Larger value responds faster but filters less noise.
// Smaller value filters more but responds more slowly.
static constexpr float FILTER_ALPHA = 0.25f;

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

static float filteredMagnitude = 0.0f;
static bool filterInitialized = false;

static uint8_t foundSampleCount = 0;

// Useful for website telemetry/debugging
static uint16_t maximumMagnitude = 0;

// --------------------------------------------------
// IR helpers
// --------------------------------------------------

static uint16_t getFoundThreshold()
{
    return USE_MAG2
        ? MAG2_FOUND_THRESHOLD
        : MAG1_FOUND_THRESHOLD;
}

static uint16_t getNearThreshold()
{
    return USE_MAG2
        ? MAG2_NEAR_THRESHOLD
        : MAG1_NEAR_THRESHOLD;
}

static bool uartDataIsFresh()
{
    const UART::Data& data = UART::getData();

    if (!data.valid)
    {
        return false;
    }

    return millis() - data.lastUpdateMs <= UART_TIMEOUT_MS;
}

static uint16_t getRawIRMagnitude()
{
    const UART::Data& data = UART::getData();

    if (!uartDataIsFresh())
    {
        return 0;
    }

    return USE_MAG2 ? data.mag2 : data.mag1;
}

static void resetDetectionFilter()
{
    filteredMagnitude = 0.0f;
    filterInitialized = false;
    foundSampleCount = 0;
    maximumMagnitude = 0;
}

static void updateDetectionFilter()
{
    if (!uartDataIsFresh())
    {
        filterInitialized = false;
        filteredMagnitude = 0.0f;
        foundSampleCount = 0;
        return;
    }

    const uint16_t rawMagnitude = getRawIRMagnitude();

    if (!filterInitialized)
    {
        filteredMagnitude = static_cast<float>(rawMagnitude);
        filterInitialized = true;
    }
    else
    {
        filteredMagnitude =
            FILTER_ALPHA * static_cast<float>(rawMagnitude) +
            (1.0f - FILTER_ALPHA) * filteredMagnitude;
    }

    const uint16_t roundedMagnitude =
        static_cast<uint16_t>(filteredMagnitude);

    if (roundedMagnitude > maximumMagnitude)
    {
        maximumMagnitude = roundedMagnitude;
    }
}

static uint16_t getFilteredMagnitude()
{
    if (!filterInitialized)
    {
        return 0;
    }

    return static_cast<uint16_t>(filteredMagnitude);
}

static bool targetWasFound()
{
    if (!filterInitialized)
    {
        foundSampleCount = 0;
        return false;
    }

    if (getFilteredMagnitude() >= getFoundThreshold())
    {
        if (foundSampleCount < REQUIRED_FOUND_SAMPLES)
        {
            ++foundSampleCount;
        }
    }
    else
    {
        // Reset if the signal falls back below the threshold.
        foundSampleCount = 0;
    }

    return foundSampleCount >= REQUIRED_FOUND_SAMPLES;
}

static int getSearchSpeed()
{
    if (getFilteredMagnitude() >= getNearThreshold())
    {
        return SEARCH_SLOW_SPEED;
    }

    return SEARCH_FAST_SPEED;
}

// --------------------------------------------------
// State transition
// --------------------------------------------------

static void changeState(AlignState newState)
{
    drive.stop();

    currentState = newState;
    stateStartTime = millis();

    // A detection must be confirmed independently in each state.
    foundSampleCount = 0;

    switch (currentState)
    {
        case AlignState::IDLE:
            break;

        case AlignState::STRAFE_RIGHT:
            drive.strafeRight(STRAFE_SPEED);
            break;

        case AlignState::SEARCH_BACKWARD:
            drive.backward(SEARCH_FAST_SPEED);
            break;

        case AlignState::SEARCH_FORWARD:
            drive.forward(SEARCH_FAST_SPEED);
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

    resetDetectionFilter();
    drive.stop();
}

void start()
{
    resetDetectionFilter();
    changeState(AlignState::STRAFE_RIGHT);
}

void update()
{
    // UART::update() must be called regularly in main.cpp.
    updateDetectionFilter();

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
        {
            const int speed = getSearchSpeed();
            drive.backward(speed);

            if (targetWasFound())
            {
                changeState(AlignState::FINISHED);
            }
            else if (
                millis() - stateStartTime >= BACKWARD_TIME_MS
            )
            {
                changeState(AlignState::SEARCH_FORWARD);
            }

            break;
        }

        case AlignState::SEARCH_FORWARD:
        {
            const int speed = getSearchSpeed();
            drive.forward(speed);

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
        }

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
    return getFilteredMagnitude();
}

uint16_t getMaximumMagnitude()
{
    return maximumMagnitude;
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