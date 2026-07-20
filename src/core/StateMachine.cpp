#include "core/StateMachine.h"

#include "tape_logic/TapeFollower.h"
#include "actuators/MecanumDrive.h"

// The actual drive object is created in main.cpp
extern MecanumDrive drive;

namespace StateMachine {

// --------------------------------------------------
// Configuration
// --------------------------------------------------

static constexpr uint16_t MAG1_THRESHOLD = 10;
static constexpr uint16_t MAG2_THRESHOLD = 10;
static constexpr int STRAFE_SPEED = 150;
static constexpr unsigned long STRAFE_TIME_MS = 1000;
static constexpr int SENSOR_SELECT_PIN = 11;

// --------------------------------------------------
// State variables
// --------------------------------------------------

static State currentState = State::TAPE_FOLLOWING;
static unsigned long stateStartTime = 0;
static bool enabled = false;

// Prevents a continuously high reading from retriggering.
static bool irTriggerArmed = true;

// --------------------------------------------------
// Internal state-change function
// --------------------------------------------------
const char* getStateName()
{
    switch (currentState)
    {
        case State::TAPE_FOLLOWING:
            return "Tape Following";

        case State::IR_STRAFE_RIGHT:
            return "IR Strafe Right";

        case State::STOPPED:
            return "Stopped";

        default:
            return "Unknown";
    }
}
void setEnabled(bool value)
{
    enabled = value;

    // Stop the robot when disabling.
    if (!enabled)
    {
        setTapeFollowing(false);
        drive.stop();
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

static void changeState(State newState)
{
    currentState = newState;
    stateStartTime = millis();

    switch (currentState) {

        case State::TAPE_FOLLOWING:
            Serial.println("State: TAPE_FOLLOWING");

            drive.stop();
            resetTapePID();
            setTapeFollowing(true);
            break;

        case State::IR_STRAFE_RIGHT:
            Serial.println("State: IR_STRAFE_RIGHT");

            // Stops tapeFollowStep() from controlling the wheels.
            setTapeFollowing(false);

            drive.strafeRight(STRAFE_SPEED);
            break;

        case State::STOPPED:
            Serial.println("State: STOPPED");

            setTapeFollowing(false);
            drive.stop();
            break;
    }
}

// --------------------------------------------------
// Public functions
// --------------------------------------------------

void begin()
{
    irTriggerArmed = true;
    changeState(State::TAPE_FOLLOWING);
    pinMode(SENSOR_SELECT_PIN, INPUT_PULLUP);
}

void update(uint16_t mag1, uint16_t mag2)
{
    if (!enabled)
    {
        return;
    }

    const bool irDetected =
        isSelectedDetected(mag1, mag2);

    if (!irDetected)
    {
        irTriggerArmed = true;
    }

    switch (currentState)
    {
        case State::TAPE_FOLLOWING:
            tapeFollowStep();

            if (irDetected && irTriggerArmed)
            {
                irTriggerArmed = false;
                changeState(State::IR_STRAFE_RIGHT);
            }
            break;

        case State::IR_STRAFE_RIGHT:
            if (millis() - stateStartTime >= STRAFE_TIME_MS)
            {
                changeState(State::STOPPED);
            }
            break;

        case State::STOPPED:
            drive.stop();
            break;
    }
}

State getState()
{
    return currentState;
}

void restart()
{
    irTriggerArmed = true;
    changeState(State::TAPE_FOLLOWING);
}
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

} // namespace StateMachine

