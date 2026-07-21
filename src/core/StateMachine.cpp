#include "core/StateMachine.h"

#include "tape_logic/TapeFollower.h"
#include "core/states/IRAligner.h"
#include "actuators/MecanumDrive.h"

// The actual drive object is created in main.cpp.
extern MecanumDrive drive;

namespace StateMachine
{

// --------------------------------------------------
// Configuration
// --------------------------------------------------

static constexpr uint16_t MAG1_THRESHOLD = 100;
static constexpr uint16_t MAG2_THRESHOLD = 20;

static constexpr int SENSOR_SELECT_PIN = 11;

// --------------------------------------------------
// State variables
// --------------------------------------------------

static State currentState = State::SLOW_TAPE_FOLLOWING;
static unsigned long stateStartTime = 0;
static bool enabled = false;

// Prevents a continuously high IR reading from retriggering.
static bool irTriggerArmed = true;
static unsigned long sideTapeTriggerCount = 0;
// --------------------------------------------------
// Internal state-change function declaration
// --------------------------------------------------

static void changeState(State newState);

// --------------------------------------------------
// State information
// --------------------------------------------------

const char* getStateName()
{
    switch (currentState)
    {
        case State::SLOW_TAPE_FOLLOWING:
            return "Slow Tape Following";

        case State::IR_ALLIGNING:
            return "IR Aligning";

        /*
        case State::RIP_SOLAR_PANEL:
            return "Rip Solar Panel";

        case State::FAST_TAPE_FOLLOWING:
            return "Fast Tape Following";

        case State::DROP_SOLAR_PANEL:
            return "Drop Solar Panel";
        */

        case State::STOPPED:
            return "Stopped";

        default:
            return "Unknown";
    }
}

// --------------------------------------------------
// Enable and disable
// --------------------------------------------------

void setEnabled(bool value)
{
    enabled = value;

    if (!enabled)
    {
        setTapeFollowing(false);
        IRAligner::stop();
        drive.stop();

        currentState = State::STOPPED;
        stateStartTime = millis();
    }
    else
    {
        restart();
    }
}

bool isEnabled()
{
    return enabled;
}

// --------------------------------------------------
// Internal state-change function
// --------------------------------------------------

static void changeState(State newState)
{
    if (newState == currentState)
    {
        return;
    }

    // Stop the behaviour from the previous state.
    setTapeFollowing(false);
    IRAligner::stop();
    drive.stop();

    currentState = newState;
    stateStartTime = millis();

    switch (currentState)
    {
        case State::SLOW_TAPE_FOLLOWING:
            Serial.println("State: SLOW_TAPE_FOLLOWING");
            setBaseSpeed(50);
            resetTapePID();
            setTapeFollowing(true);
            break;

        case State::IR_ALLIGNING:
            Serial.println("State: IR_ALLIGNING");

            IRAligner::start();
            break;

        /*
        case State::RIP_SOLAR_PANEL:
            Serial.println("State: RIP_SOLAR_PANEL");

            // SolarPanelRipper::start();
            break;

        case State::FAST_TAPE_FOLLOWING:
            Serial.println("State: FAST_TAPE_FOLLOWING");

            // resetTapePID();
            // setFastTapeFollowing(true);
            break;

        case State::DROP_SOLAR_PANEL:
            Serial.println("State: DROP_SOLAR_PANEL");

            // SolarPanelDropper::start();
            break;
        */

        case State::STOPPED:
            Serial.println("State: STOPPED");

            drive.stop();
            break;
    }
}

// --------------------------------------------------
// Public functions
// --------------------------------------------------

void begin()
{
    pinMode(SENSOR_SELECT_PIN, INPUT_PULLUP);

    irTriggerArmed = true;

    IRAligner::begin();

    // currentState initially has this value, so force a clean entry.
    currentState = State::STOPPED;
    changeState(State::SLOW_TAPE_FOLLOWING);
}

void update(uint16_t mag1, uint16_t mag2)
{
    if (!enabled)
    {
        return;
    }

    const bool irDetected = isSelectedDetected(mag1, mag2);

    // Rearm only after the selected signal goes below its threshold.
    if (!irDetected)
    {
        irTriggerArmed = true;
    }

    switch (currentState)
    {
        case State::SLOW_TAPE_FOLLOWING:
            tapeFollowStep();

            if (irDetected && irTriggerArmed)
            {
                irTriggerArmed = false;
                changeState(State::IR_ALLIGNING);
            }
            break;

        case State::IR_ALLIGNING:
            IRAligner::update();

            if (IRAligner::isFinished())
            {
                Serial.println("IR alignment successful");

                // Eventually:
                // changeState(State::RIP_SOLAR_PANEL);

                changeState(State::STOPPED);
            }
            else if (IRAligner::hasFailed())
            {
                Serial.println("IR alignment failed");

                changeState(State::STOPPED);
            }
            break;

        /*
        case State::RIP_SOLAR_PANEL:
            // SolarPanelRipper::update();

            // if (SolarPanelRipper::isFinished())
            // {
            //     changeState(State::FAST_TAPE_FOLLOWING);
            // }
            break;

        case State::FAST_TAPE_FOLLOWING:
            // fastTapeFollowStep();

            // if (nextTriggerDetected)
            // {
            //     changeState(State::DROP_SOLAR_PANEL);
            // }
            break;

        case State::DROP_SOLAR_PANEL:
            // SolarPanelDropper::update();

            // if (SolarPanelDropper::isFinished())
            // {
            //     changeState(State::STOPPED);
            // }
            break;
        */

        case State::STOPPED:
            drive.stop();
            break;
    }
}

// --------------------------------------------------
// Manual state selection
// --------------------------------------------------

bool requestStateById(const String& requestedState)
{
    State newState;

    if (requestedState == "slow_tape_following")
    {
        newState = State::SLOW_TAPE_FOLLOWING;
    }
    else if (
        requestedState == "ir_alligning" ||
        requestedState == "ir_aligning")
    {
        newState = State::IR_ALLIGNING;
    }

    /*
    else if (requestedState == "rip_solar_panel")
    {
        newState = State::RIP_SOLAR_PANEL;
    }
    else if (requestedState == "fast_tape_following")
    {
        newState = State::FAST_TAPE_FOLLOWING;
    }
    else if (requestedState == "drop_solar_panel")
    {
        newState = State::DROP_SOLAR_PANEL;
    }
    */

    else if (requestedState == "stopped")
    {
        newState = State::STOPPED;
    }
    else
    {
        return false;
    }

    // Only STOPPED may be requested while disabled.
    if (!enabled && newState != State::STOPPED)
    {
        return false;
    }

    changeState(newState);
    return true;
}

// --------------------------------------------------
// State controls
// --------------------------------------------------

State getState()
{
    return currentState;
}

void restart()
{
    irTriggerArmed = true;
    changeState(State::SLOW_TAPE_FOLLOWING);
}

// --------------------------------------------------
// IR selection
// --------------------------------------------------

bool isMag1Selected()
{
    return digitalRead(SENSOR_SELECT_PIN) == LOW;
}

uint16_t getSelectedMagnitude(uint16_t mag1, uint16_t mag2)
{
    return isMag1Selected() ? mag1 : mag2;
}

bool isSelectedDetected(uint16_t mag1, uint16_t mag2)
{
    if (isMag1Selected())
    {
        return mag1 > MAG1_THRESHOLD;
    }

    return mag2 > MAG2_THRESHOLD;
}
unsigned long getStateElapsedMs()
{
    return millis() - stateStartTime;
}

unsigned long getSideTapeTriggerCount()
{
    return sideTapeTriggerCount;
}
String getStateId()
{
    switch (currentState)
    {
        case State::SLOW_TAPE_FOLLOWING:
            return "slow_tape_following";

        case State::IR_ALLIGNING:
            return "ir_alligning";

        /*
        case State::RIP_SOLAR_PANEL:
            return "rip_solar_panel";

        case State::FAST_TAPE_FOLLOWING:
            return "fast_tape_following";

        case State::DROP_SOLAR_PANEL:
            return "drop_solar_panel";
        */

        case State::STOPPED:
            return "stopped";

        default:
            return "unknown";
    }
}

} // namespace StateMachine