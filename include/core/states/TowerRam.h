#pragma once

namespace TowerRam
{

    enum class State
    {
        IDLE,
        SHORT_STRAFE,
        ROTATE_180,
        STRAFE_RIGHT,
    
        SEARCH_STRAFE_RIGHT,
        SEARCH_LEFT_WHEELS,
    
        FINISHED
    };

void begin();
void start();
void stop();
void update();

bool isFinished();
bool isMicroswitchPressed();

} // namespace TowerRam