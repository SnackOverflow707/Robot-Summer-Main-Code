#pragma once

#include <Arduino.h>

namespace StateMachine
{

// --------------------------------------------------
// All available robot states
// --------------------------------------------------

enum class State
{
    TAPE_FOLLOW_ROCK_CHECK,

    ROCK_APPROACH, //approaches rock when reached the appropriate pose


    ROCK_METAL_CHECK,
    ROCK_GRAB,
    TAPE_FOLLOW_TO_TOWER,
    TOWER_RAM,
    TOWER_BUILD,
    RETURN_TO_TAPE,

    SLOW_TAPE_FOLLOWING,
    IR_ALIGNING,
    MANUAL_IR_ALIGNING, 

    RIP_SOLAR_PANEL,

    // Reached when IRAlignerManual's strafe + IRAligner's close-range
    // search both fail to find the beacon -- runs the fallback grab
    // sequence instead of giving up.
    RIP_SOLAR_PANEL_FALLBACK,

    ENDPOINT,
    STOPPED
};

// --------------------------------------------------
// Inputs supplied to StateMachine::update()
// --------------------------------------------------

struct Inputs
{
    uint16_t mag1;
    uint16_t mag2;

    float metalMagnitude0;
    float metalMagnitude1;


    bool sideTapeDetected;
    bool returnTapeDetected;
};

// --------------------------------------------------
// Setup and control
// --------------------------------------------------

void begin();

void setEnabled(bool value);
bool isEnabled();

void restart();
void stop();

void update(const Inputs& inputs);

// --------------------------------------------------
// Manual/website state control
// --------------------------------------------------

bool requestState(State state);
bool requestStateById(const String& stateId);

// --------------------------------------------------
// State information
// --------------------------------------------------

State getState();

const char* getStateName();
const char* getStateName(State state);

const char* getStateId();
const char* getStateId(State state);

unsigned long getStateElapsedMs();
unsigned long getCourseElapsedMs();

// This must match the uint8_t definition in StateMachine.cpp.
uint8_t getSideTapeTriggerCount();

// --------------------------------------------------
// IR sensor selection
// --------------------------------------------------

bool isMag1Selected();

uint16_t getSelectedMagnitude(
    uint16_t mag1,
    uint16_t mag2
);

bool isSelectedDetected(
    uint16_t mag1,
    uint16_t mag2
);
bool getIsMetal();

} // namespace StateMachine