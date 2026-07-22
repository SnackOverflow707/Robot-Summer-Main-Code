#include "core/StateMachine.h"

#include "tape_logic/TapeFollower.h"
#include "core/states/IRAligner.h"
#include "actuators/MecanumDrive.h"
#include "core/states/TowerRam.h"
#include "tape_logic/SideSensors.h"

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
        case State::TOWER_RAM:
            return "Tower Ram";
        case State::FIND_SIDE_TAPE:
            return "Find Side Tape";

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
        TowerRam::stop();

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

    // Stop whichever behaviour was previously running.
    setTapeFollowing(false);
    IRAligner::stop();
    TowerRam::stop();
    drive.stop();

    currentState = newState;
    stateStartTime = millis();

    switch (currentState)
    {
        case State::FIND_SIDE_TAPE:
            Serial.println("State: FIND_SIDE_TAPE");

            setBaseSpeed(50);
            resetTapePID();
            setTapeFollowing(true);
            break;

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

        case State::TOWER_RAM:
            Serial.println("State: TOWER_RAM");

            TowerRam::start();
            break;

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
    TowerRam::begin();

    currentState = State::STOPPED;
    changeState(State::FIND_SIDE_TAPE);
}
void update(uint16_t mag1, uint16_t mag2)
{
    if (!enabled)
    {
        return;
    }

    const bool irDetected = isSelectedDetected(mag1, mag2);
    const bool sideTapeTriggered = checkForSideTape();

    if (!irDetected)
    {
        irTriggerArmed = true;
    }

    switch (currentState)
    {
        case State::FIND_SIDE_TAPE:
            // Follow the normal floor tape slowly.
            tapeFollowStep();

            // Do not check IR in this state.
            if (sideTapeTriggered)
            {
                Serial.println("Side tape detected");

                sideTapeTriggerCount++;
                changeState(State::TOWER_RAM);
            }
            break;

        case State::SLOW_TAPE_FOLLOWING:
            tapeFollowStep();

            // Leave this original IR behaviour unchanged.
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
                changeState(State::STOPPED);
            }
            else if (IRAligner::hasFailed())
            {
                Serial.println("IR alignment failed");
                changeState(State::STOPPED);
            }
            break;

        case State::TOWER_RAM:
            TowerRam::update();

            if (TowerRam::isFinished())
            {
                Serial.println("Tower ram finished");
                changeState(State::STOPPED);
            }
            break;

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
    else if (requestedState == "tower_ram")
    {
        newState = State::TOWER_RAM;
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
    changeState(State::FIND_SIDE_TAPE);
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
       case State::TOWER_RAM:
            return "tower_ram";

        case State::STOPPED:
            return "stopped";

        default:
            return "unknown";
    }
}

} // namespace StateMachine