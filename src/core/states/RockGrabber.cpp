#include "core/states/RockGrabber.h"

#include "robotArm/ArmController2.h"
#include "robotArm/taskManager.h"
#include "robotArm/armSequences/rock.h"
#include "core/StateMachine.h"
#include "core/states/CourseTimeBudget.h"

// These objects must be created in main.cpp (same globals SolarPanelRipper uses).
extern ArmController2 arm;
extern TaskManager taskManager;

namespace RockGrabber
{

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
// Public functions
// --------------------------------------------------

void begin()
{
    currentState = GrabState::IDLE;
}

void start(int rockIndex)
{
    if (currentState == GrabState::RUNNING)
    {
        return;
    }

    // Defensive clamp in case the caller passes something out of range.
    if (rockIndex < 0)
    {
        rockIndex = 0;
    }
    else if (rockIndex > 5)
    {
        rockIndex = 5;
    }

    // Go/no-go: don't commit the arm to another rock if there isn't
    // enough course time left to grab it and still make the tower/panels.
    if (StateMachine::getCourseElapsedMs() + ROCK_GRAB_WORST_CASE_MS >
        ROCK_TIME_DEADLINE_MS)
    {
        currentState = GrabState::FAILED;
        return;
    }

    currentState = GrabState::RUNNING;

    // Blocking: each of these runs to completion before returning.
    // See TaskManager::executeMove / objectGripCheckSequence.
    rockReachSequence(taskManager, rockIndex);

    // DEPRECATED: no microswitch on finger 
    /*
    const bool gripped =
        taskManager.objectGripCheckSequence(
            ROCK_POSITIONS[rockIndex].grab,
            ROCK_NATTEMPTS);

    if (gripped)
    {
        rockRetractSequence(taskManager);
        currentState = GrabState::FINISHED;
    }
    else
    {
        arm.openClaw();
        currentState = GrabState::FAILED;
    }
     */
}

void update()
{
    // Nothing to do here while rockSequence() is blocking -
    // start() already ran the whole thing synchronously.
}

void stop()
{
    // TODO: add a real arm-stop / retract-to-safe-position call here
    // once ArmController2 supports interrupting mid-move.
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