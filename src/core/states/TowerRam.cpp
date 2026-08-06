#include "core/states/TowerRam.h"

#include <Arduino.h>

#include "actuators/MecanumDrive.h"

extern MecanumDrive drive;

namespace TowerRam
{

// --------------------------------------------------
// Configuration
// --------------------------------------------------

static constexpr int ROTATE_SPEED = 140;
static constexpr unsigned long ROTATE_TIME_MS = 3400;

static constexpr int STRAFE_SPEED = 120;
static constexpr unsigned long SHORT_STRAFE_TIME_MS = 800;
static constexpr unsigned long STRAFE_TIME_MS = 1000;

// Alternating search movement
static constexpr int LEFT_WHEELS_SPEED = 120;
static constexpr unsigned long SEARCH_STRAFE_TIME_MS = 500;
static constexpr unsigned long LEFT_WHEELS_TIME_MS = 500;

// Maximum TOTAL time spent searching for the microswitch
static constexpr unsigned long SEARCH_TIMEOUT_MS = 5000;

// Microswitch
static constexpr int MICROSWITCH_PIN = 42;


// --------------------------------------------------
// State variables
// --------------------------------------------------

static State currentState = State::IDLE;
static unsigned long stateStartTime = 0;

// Does NOT reset when alternating between the two search states.
// This gives us a 5-second total search timeout.
static unsigned long searchStartTime = 0;


// --------------------------------------------------
// Helpers
// --------------------------------------------------

static void changeState(State newState)
{
    currentState = newState;
    stateStartTime = millis();
}


static bool microswitchPressed()
{
    // INPUT_PULLUP:
    // HIGH = not pressed
    // LOW  = pressed
    return digitalRead(MICROSWITCH_PIN) == LOW;
}


// --------------------------------------------------
// Public functions
// --------------------------------------------------

void begin()
{
    pinMode(MICROSWITCH_PIN, INPUT_PULLUP);

    currentState = State::IDLE;
    stateStartTime = 0;
    searchStartTime = 0;
}


void start()
{
    drive.stop();

    searchStartTime = 0;

    changeState(State::SHORT_STRAFE);
}


void stop()
{
    drive.stop();

    searchStartTime = 0;

    changeState(State::IDLE);
}


void update()
{
    const unsigned long elapsed = millis() - stateStartTime;

    switch (currentState)
    {
        // --------------------------------------------------
        // Idle
        // --------------------------------------------------

        case State::IDLE:
        {
            drive.stop();
            break;
        }


        // --------------------------------------------------
        // Initial short strafe
        // --------------------------------------------------

        case State::SHORT_STRAFE:
        {
            drive.strafeRight(STRAFE_SPEED);

            if (elapsed >= SHORT_STRAFE_TIME_MS)
            {
                drive.stop();
                changeState(State::ROTATE_180);
            }

            break;
        }


        // --------------------------------------------------
        // 180 degree rotation
        // --------------------------------------------------

        case State::ROTATE_180:
        {
            drive.rotateCounterClockwise(ROTATE_SPEED);

            if (elapsed >= ROTATE_TIME_MS)
            {
                drive.stop();
                changeState(State::STRAFE_RIGHT);
            }

            break;
        }


        // --------------------------------------------------
        // Main strafe right
        // --------------------------------------------------

        case State::STRAFE_RIGHT:
        {
            drive.strafeRight(STRAFE_SPEED);

            if (elapsed >= STRAFE_TIME_MS)
            {
                drive.stop();

                // Start the 5-second TOTAL search timer here.
                searchStartTime = millis();

                changeState(State::SEARCH_STRAFE_RIGHT);
            }

            break;
        }


        // --------------------------------------------------
        // Search: strafe right
        // --------------------------------------------------

        case State::SEARCH_STRAFE_RIGHT:
        {
            // Microswitch found
            if (microswitchPressed())
            {
                drive.stop();
                changeState(State::FINISHED);
                break;
            }

            // Total search has exceeded 5 seconds
            if (millis() - searchStartTime >= SEARCH_TIMEOUT_MS)
            {
                drive.stop();
                changeState(State::FINISHED);
                break;
            }

            drive.strafeRight(STRAFE_SPEED);

            // Alternate to moving the left wheels
            if (elapsed >= SEARCH_STRAFE_TIME_MS)
            {
                drive.stop();
                changeState(State::SEARCH_LEFT_WHEELS);
            }

            break;
        }


        // --------------------------------------------------
        // Search: both left wheels forward
        // --------------------------------------------------

        case State::SEARCH_LEFT_WHEELS:
        {
            // Microswitch found
            if (microswitchPressed())
            {
                drive.stop();
                changeState(State::FINISHED);
                break;
            }

            // Total search has exceeded 5 seconds
            if (millis() - searchStartTime >= SEARCH_TIMEOUT_MS)
            {
                drive.stop();
                changeState(State::FINISHED);
                break;
            }

            drive.leftWheelsForward(LEFT_WHEELS_SPEED);

            // Alternate back to strafing right
            if (elapsed >= LEFT_WHEELS_TIME_MS)
            {
                drive.stop();
                changeState(State::SEARCH_STRAFE_RIGHT);
            }

            break;
        }


        // --------------------------------------------------
        // Finished
        // --------------------------------------------------

        case State::FINISHED:
        {
            drive.stop();
            break;
        }
    }
}


bool isFinished()
{
    return currentState == State::FINISHED;
}
bool isMicroswitchPressed()
{
    return microswitchPressed();
}

} // namespace TowerRam