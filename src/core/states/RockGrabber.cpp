#include "core/states/RockGrabber.h"

#include <Arduino.h>

#include "actuators/MecanumDrive.h"
#include "robotArm/ArmController2.h"
#include "robotArm/taskManager.h"
#include "robotArm/armSequences/rock.h"
#include "core/StateMachine.h"
#include "core/states/CourseTimeBudget.h"
#include "core/states/RockApproach.h"
#include "tape_logic/TapeFollower.h"

extern MecanumDrive drive;
extern ArmController2 arm;
extern TaskManager taskManager;

namespace RockGrabber
{

// --------------------------------------------------
// Configuration
// --------------------------------------------------

static constexpr int CORRECTION_STRAFE_SPEED = 100;
static constexpr unsigned long CORRECTION_STRAFE_TIME_MS = 600;

// Small pause after stopping the drivetrain so the robot is not
// still rocking when the arm starts moving.
static constexpr unsigned long DRIVE_SETTLE_TIME_MS = 150;

// Give the claw time to fully close before lifting the rock.
static constexpr unsigned long CLAW_CLOSE_TIME_MS = 500;

// --------------------------------------------------
// Internal states
// --------------------------------------------------

enum class GrabState
{
    IDLE,
    RUNNING,
    FINISHED,
    FAILED
};

static GrabState currentState = GrabState::IDLE;

// --------------------------------------------------
// Helpers
// --------------------------------------------------

static int clampRockIndex(int rockIndex)
{
    if (rockIndex < 0)
    {
        return 0;
    }

    if (rockIndex > 5)
    {
        return 5;
    }

    return rockIndex;
}

/*
 * RockApproach used:
 *
 *     coil 0 = left coil
 *     coil 1 = right coil
 *
 * Your rock ordering was:
 *
 *     rock 0 = right
 *     rock 1 = left
 *     rock 2 = right
 *     rock 3 = left
 *     rock 4 = right
 *     rock 5 = left
 *
 * Therefore even indices are right-side rocks and odd indices
 * are left-side rocks.
 */
static bool isRightSideRock(int rockIndex)
{
    return (rockIndex % 2) == 0;
}

/*
 * This deliberately strafes opposite the direction used to approach
 * the rock.
 *
 * Right-side rock:
 *     Approach moved toward the right side.
 *     Grab correction moves left.
 *
 * Left-side rock:
 *     Approach moved toward the left side.
 *     Grab correction moves right.
 *
 * Reverse these two calls if your physical drivetrain directions
 * are opposite.
 */
static void correctionStrafe(int rockIndex)
{
    if (isRightSideRock(rockIndex))
    {
        drive.strafeLeft(CORRECTION_STRAFE_SPEED);
    }
    else
    {
        drive.strafeRight(CORRECTION_STRAFE_SPEED);
    }

    delay(CORRECTION_STRAFE_TIME_MS);

    drive.stop();
    delay(DRIVE_SETTLE_TIME_MS);
}

// --------------------------------------------------
// Public functions
// --------------------------------------------------

void begin()
{
    currentState = GrabState::IDLE;
}

void start(int rockIndex)
{
    const RockApproach::RockPos* rockPositions =
    RockApproach::getRockPositions();

const bool rockIsRight =
    rockPositions[rockIndex].coil == 1;

// Strafe opposite the RockApproach direction.
if (rockIsRight)
{
    drive.strafeLeft(CORRECTION_STRAFE_SPEED);
}
else
{
    drive.strafeRight(CORRECTION_STRAFE_SPEED);
}

delay(600);
drive.stop();
delay(150);

// Reach and close the claw.
if (rockIsRight)
{
    taskManager.executeMove(RIGHT_ROCK_GRAB_OPEN);
    taskManager.executeMove(RIGHT_ROCK_GRAB_CLOSED);
}
else
{
    taskManager.executeMove(LEFT_ROCK_GRAB_OPEN);
    taskManager.executeMove(LEFT_ROCK_GRAB_CLOSED);
}

// Lift and place the rock.
taskManager.executeMove(ROCK_LIFT);
taskManager.executeMove(ROCK_OVER_POST);
taskManager.executeMove(ROCK_PLACE);

arm.openClaw();
delay(400);

taskManager.executeMove(ROCK_RETRACT);

// Strafe back toward the tape.
while (true)
{
    updateTapeSensors();

    const TapeFollowerStatus status = getTapeFollowerStatus();

    const bool onTape =
        !status.leftWhite ||
        !status.rightWhite;

    if (onTape)
    {
        break;
    }

    if (rockIsRight)
    {
        // We moved left to grab the rock,
        // so move right to return.
        drive.strafeRight(150);
    }
    else
    {
        // We moved right to grab the rock,
        // so move left to return.
        drive.strafeLeft(150);
    }

    delay(10);
}

drive.stop();
currentState = GrabState::FINISHED;
}

void update()
{
    // The sequence is currently blocking, so everything is completed
    // inside start().
}

void stop()
{
    drive.stop();

    // Arm motion cannot currently be interrupted through TaskManager.
    currentState = GrabState::IDLE;
}

bool isFinished()
{
    return currentState == GrabState::FINISHED;
}

bool hasFailed()
{
    return currentState == GrabState::FAILED;
}

bool isDone()
{
    return isFinished() || hasFailed();
}

} // namespace RockGrabber