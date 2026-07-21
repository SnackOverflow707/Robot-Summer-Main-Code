#pragma once

namespace IRAligner
{
    void begin();
    void start();
    void update();
    void stop();

    bool isFinished();
    bool hasFailed();
    bool isDone();

    const char* getStateName();
}