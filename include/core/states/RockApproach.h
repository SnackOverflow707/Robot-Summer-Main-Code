#pragma once
#include <Arduino.h>

namespace RockApproach 
{
    void begin();
    void start(uint8_t rockIndex);
    void update();
    void stop();

    bool isFinished();
    bool hasFailed();

    struct RockPos {
        float x;
        float y;
        uint8_t coil; //0 if left sensor has to be used, 1 if right sensor has to be used(as viewed from the rear of the robot)
    };

    extern const RockPos ROCK_POSITIONS[6];
}