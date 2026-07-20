#pragma once

#include <Arduino.h>

namespace StateMachine
{

enum class State
{
    TAPE_FOLLOWING,
    IR_STRAFE_RIGHT,
    STOPPED
};

void begin();
void update(uint16_t mag1, uint16_t mag2);

void setEnabled(bool enabled);
bool isEnabled();

State getState();
const char* getStateName();

void restart();

bool isMag1Selected();

uint16_t getSelectedMagnitude(
    uint16_t mag1,
    uint16_t mag2
);

bool isSelectedDetected(
    uint16_t mag1,
    uint16_t mag2
);

} // namespace StateMachine