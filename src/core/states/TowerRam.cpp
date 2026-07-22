#include "core/states/TowerRam.h"

#include <Arduino.h>

#include "actuators/MecanumDrive.h"

extern MecanumDrive drive;

namespace TowerRam
{

static State currentState = State::IDLE;
static unsigned long stateStartTime = 0;

static constexpr int ROTATE_SPEED = 140;
static constexpr unsigned long ROTATE_TIME_MS = 3500;

static constexpr int STRAFE_SPEED = 120;
static constexpr unsigned long SHORT_STRAFE_TIME_MS = 500;
static constexpr unsigned long STRAFE_TIME_MS = 1000;
static void changeState(State newState)
{
    currentState = newState;
    stateStartTime = millis();
}


void begin()
{
    currentState = State::IDLE;
    stateStartTime = 0;
}


void start()
{
    drive.stop();
    changeState(State::SHORT_STRAFE);
}


void stop()
{
    drive.stop();
    changeState(State::IDLE);
}


void update()
{
    const unsigned long elapsed = millis() - stateStartTime;

    switch (currentState)
    {
        case State::IDLE:
            drive.stop();
            break;


        case State::SHORT_STRAFE:

            drive.strafeRight(STRAFE_SPEED);

            if (elapsed >= SHORT_STRAFE_TIME_MS)
            {
                drive.stop();
                changeState(State::ROTATE_180);
            }

            break;


        case State::ROTATE_180:

            drive.rotateCounterClockwise(ROTATE_SPEED);

            if (elapsed >= ROTATE_TIME_MS)
            {
                drive.stop();
                changeState(State::STRAFE_RIGHT);
            }

            break;


        case State::STRAFE_RIGHT:

            drive.strafeRight(STRAFE_SPEED);

            if (elapsed >= STRAFE_TIME_MS)
            {
                drive.stop();
                changeState(State::FINISHED);
            }

            break;


        case State::FINISHED:

            drive.stop();
            break;
    }
}


bool isFinished()
{
    return currentState == State::FINISHED;
}

} // namespace TowerRam