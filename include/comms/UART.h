#pragma once

#include <Arduino.h>

namespace UART
{

struct Data
{
    uint16_t mag1;
    uint16_t mag2;
    uint8_t mask;

    uint32_t frameCount;
    unsigned long lastUpdateMs;
    bool valid;
};

struct MetalData
{
    float frequencyHz;

    uint32_t frameCount;
    unsigned long lastUpdateMs;
    bool valid;
};

struct PoseData
{
    float x;
    float y;
    float theta;

    float vx;
    float vy;
    float omega;

    uint32_t frameCount;
    unsigned long lastUpdateMs;
    bool valid;
};

PoseData getPoseData();
void resetFlowPose();



void begin();
void update();

Data getData();

// Return the requested metal detector.
MetalData getMetalData(uint8_t detectorId);

bool isMag1Selected();
bool isMag2Selected();

uint16_t getSelectedMagnitude();
uint8_t getSelectedFrequency();
bool isSelectedDetected();


} // namespace UART