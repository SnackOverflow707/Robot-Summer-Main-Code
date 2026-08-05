#ifndef ROCK_METAL_CHECK_H
#define ROCK_METAL_CHECK_H

#include <Arduino.h>

#include "core/StateMachine.h"

namespace RockMetalCheck
{

void begin();
void resetBaselines();

// Call this near the start of the course until it returns true.
void updateBaselines(const StateMachine::Inputs& inputs);

bool areBaselinesReady();

void start(uint8_t rockIndex);
void update(const StateMachine::Inputs& inputs);
void stop();

bool isFinished();
bool hasFailed();
bool metalFound();

// Website telemetry
float getBaseline0();
float getBaseline1();

float getLatestCheckValue();
float getLatestBaselineValue();
float getLatestChange();

uint8_t getLatestRockIndex();
uint8_t getLatestCoil();

}

#endif