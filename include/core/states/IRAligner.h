#pragma once

namespace IRAligner
{
    void begin();

    // skipInitialStrafe: true when the robot has already been positioned
    // near the panel (e.g. by IRAlignerManual's hardcoded
    // strafe) -- goes straight to the backward/forward peak search
    // instead of doing another blind strafe right first.
    void start(bool skipInitialStrafe = false);
    void update();
    void stop();
    void alignRobot(); 

    bool isFinished();
    bool hasFailed();
    bool isDone();

    const char* getStateName();
}