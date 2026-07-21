#pragma once

#include <Arduino.h>

namespace StateMachine
{

enum class State
{
    SLOW_TAPE_FOLLOWING,
    IR_ALLIGNING,

    /*
    RIP_SOLAR_PANEL,
    FAST_TAPE_FOLLOWING,
    DROP_SOLAR_PANEL,
    */

    STOPPED
};

void begin();
void update(uint16_t mag1, uint16_t mag2);

void setEnabled(bool value);
bool isEnabled();

bool requestStateById(const String& requestedState);

State getState();
const char* getStateName();

void restart();

bool isMag1Selected();
uint16_t getSelectedMagnitude(uint16_t mag1, uint16_t mag2);
bool isSelectedDetected(uint16_t mag1, uint16_t mag2);
unsigned long getStateElapsedMs();
unsigned long getSideTapeTriggerCount();
String getStateId();
} // namespace StateMachine