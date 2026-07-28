#include "core/states/TowerBuilder.h"

#include "robotArm/armSequences/tower.h"
#include "robotArm/taskManager.h"


extern TaskManager taskManager;


namespace TowerBuilder
{

static bool running = false;
static bool finished = false;
static bool failed = false;


void begin()
{
    running = false;
    finished = false;
    failed = false;
}


void start()
{
    if (running)
    {
        return;
    }

    running = true;
    finished = false;
    failed = false;

    Serial.println("Starting tower build");

    towerSequence(taskManager);

    Serial.println("Tower build finished");

    running = false;
    finished = true;
}


void update()
{
    // towerSequence() currently executes the entire sequence
    // inside start(), so there is nothing to update here.
}


void stop()
{
    running = false;
}


bool isFinished()
{
    return finished;
}


bool hasFailed()
{
    return failed;
}

} // namespace TowerBuilder