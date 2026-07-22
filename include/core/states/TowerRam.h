#pragma once

namespace TowerRam
{

enum class State
{
    IDLE,
    SHORT_STRAFE,
    ROTATE_180,
    STRAFE_RIGHT,
    FINISHED
};

void begin();
void start();
void stop();
void update();

bool isFinished();

} // namespace TowerRam