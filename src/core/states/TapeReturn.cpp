#include "core/states/TapeReturn.h"

#include <Arduino.h>

#include "actuators/MecanumDrive.h"
#include "tape_logic/TapeFollower.h"

extern MecanumDrive drive;

namespace TapeReturn
{

enum class ReturnState
{
    IDLE,
    STRAFE_LEFT,
    ROTATE_CLOCKWISE,
    FIND_TAPE,
    FINISHED
};

static ReturnState currentState = ReturnState::IDLE;
static unsigned long stateStartTime = 0;


// --------------------------------------------------
// Configuration
// --------------------------------------------------

static constexpr int STRAFE_SPEED = 120;
static constexpr int ROTATE_SPEED = 140;

static constexpr unsigned long STRAFE_LEFT_TIME_MS = 2000;
static constexpr unsigned long ROTATE_TIME_MS = 2300;


// --------------------------------------------------
// State helper
// --------------------------------------------------

static void changeState(ReturnState newState)
{
    currentState = newState;
    stateStartTime = millis();
}


// --------------------------------------------------
// Public functions
// --------------------------------------------------

void begin()
{
    drive.stop();

    currentState = ReturnState::IDLE;
    stateStartTime = 0;
}


void start()
{
    drive.stop();

    changeState(ReturnState::STRAFE_LEFT);
}


void update()
{
    switch (currentState)
    {
        case ReturnState::IDLE:
            drive.stop();
            break;


        // ------------------------------------------
        // 1. Strafe left for 2000 ms
        // ------------------------------------------

        case ReturnState::STRAFE_LEFT:

            drive.strafeLeft(STRAFE_SPEED);

            if (millis() - stateStartTime >= STRAFE_LEFT_TIME_MS)
            {
                drive.stop();
                changeState(ReturnState::ROTATE_CLOCKWISE);
            }

            break;


        // ------------------------------------------
        // 2. Rotate clockwise for 1000 ms
        // ------------------------------------------

        case ReturnState::ROTATE_CLOCKWISE:

            drive.rotateCounterClockwise(ROTATE_SPEED);

            if (millis() - stateStartTime >= ROTATE_TIME_MS)
            {
                drive.stop();
                changeState(ReturnState::FIND_TAPE);
            }

            break;


        // ------------------------------------------
        // 3. Strafe right until either front
        //    tape sensor detects tape
        // ------------------------------------------

        case ReturnState::FIND_TAPE:
        {
            const TapeFollowerStatus tapeStatus =
                getTapeFollowerStatus();

            // If either front sensor sees tape,
            // we are back at the line.
            if (tapeStatus.leftWhite || tapeStatus.rightWhite)
            {
                drive.stop();
                changeState(ReturnState::FINISHED);
                break;
            }

            drive.strafeRight(STRAFE_SPEED);

            break;
        }


        case ReturnState::FINISHED:
            drive.stop();
            break;
    }
}


void stop()
{
    drive.stop();
    changeState(ReturnState::IDLE);
}


bool isFinished()
{
    return currentState == ReturnState::FINISHED;
}

} // namespace TapeReturn