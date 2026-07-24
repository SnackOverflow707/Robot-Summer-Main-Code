#include "core/states/SolarPanelRipper.h"

#include "robotArm/ArmController2.h"
#include "robotArm/taskManager.h"
#include "robotArm/armSequences/solarPanels.h"

// These objects must be created in main.cpp.
extern ArmController2 arm;
extern TaskManager taskManager;

namespace SolarPanelRipper
{

enum class RipperState
{
    IDLE,
    RUNNING,
    FINISHED,
    FAILED
};

static RipperState currentState = RipperState::IDLE;

void begin()
{
    currentState = RipperState::IDLE;
}

void start()
{
    if (currentState == RipperState::RUNNING)
    {
        return;
    }

    currentState = RipperState::RUNNING;

    // This currently runs the entire arm sequence before returning.
    solarPanelSequence(taskManager);

    currentState = RipperState::FINISHED;
}

void update()
{
    // Nothing is required here while solarPanelSequence() is blocking.
}

void stop()
{
    // Add a real arm-stop function here if ArmController2 supports one.
    currentState = RipperState::IDLE;
}

bool isFinished()
{
    return currentState == RipperState::FINISHED;
}

bool hasFailed()
{
    return currentState == RipperState::FAILED;
}

bool isDone()
{
    return isFinished() || hasFailed();
}

} // namespace SolarPanelRipper