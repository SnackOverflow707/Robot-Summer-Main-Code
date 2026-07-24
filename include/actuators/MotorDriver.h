#pragma once

#include <Arduino.h>
#include "driver/mcpwm.h"

class MotorDriver
{
public:
    MotorDriver(
        int pinA,
        int pinB,
        mcpwm_unit_t unit,
        mcpwm_timer_t timer
    );

    void begin();
    void setSpeed(int speed);
    void stop();

private:
    int _pinA;
    int _pinB;

    mcpwm_unit_t _unit;
    mcpwm_timer_t _timer;

    int _lastSpeed;
};