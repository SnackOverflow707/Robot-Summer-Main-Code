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


// --------------------------------------------------
// Search motion
// --------------------------------------------------

// First move backward far enough to get to one side
// of the directional IR peak.
static constexpr int BACKWARD_SPEED = 60;
static constexpr unsigned long BACKWARD_TIME_MS = 2000;

// Then scan forward slowly across the IR peak.
static constexpr int FORWARD_SCAN_SPEED = 30;

// Maximum time allowed for the forward peak search.
static constexpr unsigned long FORWARD_SEARCH_TIME_MS = 4000;


// --------------------------------------------------
// Peak detection
// --------------------------------------------------

// Signal must reach at least this value before we consider
// the observed maximum to be a real IR peak.
//
// These are NOT "stop here" thresholds.
// They only reject noise / weak background signals.
static constexpr uint16_t MAG1_MIN_VALID_PEAK = 10000;
static constexpr uint16_t MAG2_MIN_VALID_PEAK = 3000;

// Once the signal falls below this fraction of the best
// signal seen, we assume we have passed the peak.
//
// Example:
// peak = 3000
// 0.75 * 3000 = 2250
//
// Once the signal is below 2250 for several samples,
// we consider the peak passed.
static constexpr float PEAK_DROP_RATIO = 0.75f;

// Require several consecutive falling readings so one
// noisy sample does not cause a reversal.
static constexpr uint8_t REQUIRED_FALLING_SAMPLES = 4;


// --------------------------------------------------
// Return to peak
// --------------------------------------------------

// Use the same speed in reverse so time is approximately
// proportional to distance.
static constexpr int RETURN_SPEED = FORWARD_SCAN_SPEED;

// Don't allow an unexpectedly huge return movement.
static constexpr unsigned long MAX_RETURN_TIME_MS = 1200;


// --------------------------------------------------
// UART
// --------------------------------------------------

static constexpr unsigned long UART_TIMEOUT_MS = 250;


// --------------------------------------------------
// Filtering
// --------------------------------------------------

// Exponential smoothing.
//
// Higher -> reacts faster, more noise
// Lower  -> smoother, more lag
static constexpr float FILTER_ALPHA = 0.30f;


// --------------------------------------------------
// IR channel selection
// --------------------------------------------------

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

    // Move to one side of the directional peak.
    SEARCH_BACKWARD,

    // Slowly sweep across the peak.
    SEARCH_FORWARD,

    // We passed the peak, so reverse approximately
    // back to where the maximum occurred.
    RETURN_TO_PEAK,

    FINISHED,
    NOT_FOUND
};


static AlignState currentState = AlignState::IDLE;

static unsigned long stateStartTime = 0;


// --------------------------------------------------
// Filter state
// --------------------------------------------------

static float filteredMagnitude = 0.0f;
static bool filterInitialized = false;


// --------------------------------------------------
// Peak tracking
// --------------------------------------------------

static uint16_t maximumMagnitude = 0;

// Time at which the strongest signal was observed
// during the forward scan.
static unsigned long peakTime = 0;

// How long we continued forward after the peak.
// We reverse for approximately this long.
static unsigned long returnTimeMs = 0;

// Consecutive samples sufficiently below the peak.
static uint8_t fallingSampleCount = 0;

// True once we've seen a signal strong enough to
// consider the peak real.
static bool validPeakSeen = false;


// --------------------------------------------------
// IR helpers
// --------------------------------------------------

static uint16_t getMinimumValidPeak()
{
    return USE_MAG2
        ? MAG2_MIN_VALID_PEAK
        : MAG1_MIN_VALID_PEAK;
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

    return USE_MAG2
        ? data.mag2
        : data.mag1;
}


// --------------------------------------------------
// Filter
// --------------------------------------------------

static void resetDetectionFilter()
{
    filteredMagnitude = 0.0f;
    filterInitialized = false;
}


static void updateDetectionFilter()
{
    if (!uartDataIsFresh())
    {
        filterInitialized = false;
        filteredMagnitude = 0.0f;
        return;
    }

    const uint16_t rawMagnitude =
        getRawIRMagnitude();

    if (!filterInitialized)
    {
        filteredMagnitude =
            static_cast<float>(rawMagnitude);

        filterInitialized = true;
    }
    else
    {
        filteredMagnitude =
            FILTER_ALPHA *
                static_cast<float>(rawMagnitude)
            +
            (1.0f - FILTER_ALPHA) *
                filteredMagnitude;
    }
}


static uint16_t getFilteredMagnitude()
{
    if (!filterInitialized)
    {
        return 0;
    }

    return static_cast<uint16_t>(
        filteredMagnitude
    );
}


// --------------------------------------------------
// Peak tracking
// --------------------------------------------------

static void resetPeakTracking()
{
    maximumMagnitude = 0;

    peakTime = 0;
    returnTimeMs = 0;

    fallingSampleCount = 0;

    validPeakSeen = false;
}


static bool updatePeakTracking()
{
    if (!filterInitialized)
    {
        fallingSampleCount = 0;
        return false;
    }

    const uint16_t magnitude =
        getFilteredMagnitude();


    // --------------------------------------------------
    // New maximum found
    // --------------------------------------------------

    if (magnitude > maximumMagnitude)
    {
        maximumMagnitude = magnitude;

        peakTime = millis();

        fallingSampleCount = 0;

        if (
            maximumMagnitude >=
            getMinimumValidPeak()
        )
        {
            validPeakSeen = true;
        }

        return false;
    }


    // --------------------------------------------------
    // Don't look for a falling edge until we've seen
    // a legitimate peak.
    // --------------------------------------------------

    if (!validPeakSeen)
    {
        return false;
    }


    // --------------------------------------------------
    // Determine how far below the peak we currently are
    // --------------------------------------------------

    const float fallingThreshold =
        static_cast<float>(maximumMagnitude) *
        PEAK_DROP_RATIO;


    if (
        static_cast<float>(magnitude) <
        fallingThreshold
    )
    {
        if (
            fallingSampleCount <
            REQUIRED_FALLING_SAMPLES
        )
        {
            ++fallingSampleCount;
        }
    }
    else
    {
        fallingSampleCount = 0;
    }


    // --------------------------------------------------
    // Peak has been passed
    // --------------------------------------------------

    if (
        fallingSampleCount >=
        REQUIRED_FALLING_SAMPLES
    )
    {
        returnTimeMs =
            millis() - peakTime;

        // Limit the reverse correction.
        if (
            returnTimeMs >
            MAX_RETURN_TIME_MS
        )
        {
            returnTimeMs =
                MAX_RETURN_TIME_MS;
        }

        return true;
    }


    return false;
}


// --------------------------------------------------
// State transition
// --------------------------------------------------

static void changeState(AlignState newState)
{
    drive.stop();

    currentState = newState;
    stateStartTime = millis();


    switch (currentState)
    {
        // --------------------------------------------------

        case AlignState::IDLE:
        {
            break;
        }


        // --------------------------------------------------

        case AlignState::STRAFE_RIGHT:
        {
            drive.strafeRight(
                STRAFE_SPEED
            );

            break;
        }


        // --------------------------------------------------

        case AlignState::SEARCH_BACKWARD:
        {
            drive.backward(
                BACKWARD_SPEED
            );

            break;
        }


        // --------------------------------------------------

        case AlignState::SEARCH_FORWARD:
        {
            // Important:
            // reset peak tracking immediately before
            // the actual precision sweep.
            resetPeakTracking();

            drive.forward(
                FORWARD_SCAN_SPEED
            );

            break;
        }


        // --------------------------------------------------

        case AlignState::RETURN_TO_PEAK:
        {
            drive.backward(
                RETURN_SPEED
            );

            break;
        }


        // --------------------------------------------------

        case AlignState::FINISHED:
        case AlignState::NOT_FOUND:
        {
            drive.stop();
            break;
        }
    }
}


// --------------------------------------------------
// Public functions
// --------------------------------------------------

void begin()
{
    currentState =
        AlignState::IDLE;

    stateStartTime = 0;

    resetDetectionFilter();
    resetPeakTracking();

    drive.stop();
}


void start()
{
    resetDetectionFilter();
    resetPeakTracking();

    changeState(
        AlignState::STRAFE_RIGHT
    );
}


// --------------------------------------------------
// Update
// --------------------------------------------------

void update()
{
    // UART::update() must still be called regularly
    // from main.cpp.
    updateDetectionFilter();


    switch (currentState)
    {
        // --------------------------------------------------

        case AlignState::IDLE:
        {
            break;
        }


        // --------------------------------------------------
        // Initial lateral positioning
        // --------------------------------------------------

        case AlignState::STRAFE_RIGHT:
        {
            if (
                millis() - stateStartTime >=
                STRAFE_TIME_MS
            )
            {
                changeState(
                    AlignState::SEARCH_BACKWARD
                );
            }

            break;
        }


        // --------------------------------------------------
        // Move backward to get to one side of the peak
        // --------------------------------------------------

        case AlignState::SEARCH_BACKWARD:
        {
            drive.backward(
                BACKWARD_SPEED
            );

            if (
                millis() - stateStartTime >=
                BACKWARD_TIME_MS
            )
            {
                changeState(
                    AlignState::SEARCH_FORWARD
                );
            }

            break;
        }


        // --------------------------------------------------
        // Precision forward sweep
        // --------------------------------------------------

        case AlignState::SEARCH_FORWARD:
        {
            drive.forward(
                FORWARD_SCAN_SPEED
            );


            // Track the maximum magnitude and wait until
            // we've clearly moved past it.
            if (updatePeakTracking())
            {
                changeState(
                    AlignState::RETURN_TO_PEAK
                );

                break;
            }


            // Failed to find a real peak.
            if (
                millis() - stateStartTime >=
                FORWARD_SEARCH_TIME_MS
            )
            {
                changeState(
                    AlignState::NOT_FOUND
                );
            }


            break;
        }


        // --------------------------------------------------
        // Reverse back toward the measured peak
        // --------------------------------------------------

        case AlignState::RETURN_TO_PEAK:
        {
            drive.backward(
                RETURN_SPEED
            );


            if (
                millis() - stateStartTime >=
                returnTimeMs
            )
            {
                changeState(
                    AlignState::FINISHED
                );
            }


            break;
        }


        // --------------------------------------------------

        case AlignState::FINISHED:
        case AlignState::NOT_FOUND:
        {
            drive.stop();
            break;
        }
    }
}


// --------------------------------------------------
// Stop
// --------------------------------------------------

void stop()
{
    changeState(
        AlignState::IDLE
    );
}


// --------------------------------------------------
// Status
// --------------------------------------------------

bool isFinished()
{
    return
        currentState ==
        AlignState::FINISHED;
}


bool hasFailed()
{
    return
        currentState ==
        AlignState::NOT_FOUND;
}


bool isDone()
{
    return
        isFinished() ||
        hasFailed();
}


// --------------------------------------------------
// Telemetry
// --------------------------------------------------

uint16_t getCurrentMagnitude()
{
    return getFilteredMagnitude();
}


uint16_t getMaximumMagnitude()
{
    return maximumMagnitude;
}


// --------------------------------------------------
// State name
// --------------------------------------------------

const char* getStateName()
{
    switch (currentState)
    {
        case AlignState::IDLE:
            return "Idle";

        case AlignState::STRAFE_RIGHT:
            return "Strafe Right";

        case AlignState::SEARCH_BACKWARD:
            return "Move to Start";

        case AlignState::SEARCH_FORWARD:
            return "Peak Scan";

        case AlignState::RETURN_TO_PEAK:
            return "Return to Peak";

        case AlignState::FINISHED:
            return "IR Aligned";

        case AlignState::NOT_FOUND:
            return "IR Peak Not Found";
    }

    return "Unknown";
}

} // namespace IRAligner