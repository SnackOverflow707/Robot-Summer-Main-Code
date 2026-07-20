#pragma once

#include <Arduino.h>

namespace UART
{

struct Data
{
    uint16_t mag1;
    uint16_t mag2;
    uint8_t mask;

    unsigned long frameCount;
    unsigned long lastUpdateMs;

    bool valid;
};

void begin();
void update();

Data getData();

bool isMag1Selected();
bool isMag2Selected();

uint16_t getSelectedMagnitude();
uint8_t getSelectedFrequency();
bool isSelectedDetected();

} // namespace UART