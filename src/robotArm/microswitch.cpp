#include "microswitch.h"
#include <Arduino.h>

MicroSwitch::MicroSwitch(int pin) : 
    _pin(pin) 
{}

void MicroSwitch::begin() {
    pinMode(_pin, INPUT_PULLUP);
}

bool MicroSwitch::isPressed() const {
    return digitalRead(_pin) == LOW; //wired with NO -> GND and COM -> GPIO 
}

//checks if the claw has been fully closed for the specified time period. 
bool MicroSwitch::fullyClosed(int durationMs) {
    unsigned long start = millis();
    while (isPressed()) {
        if (millis() - start >= (unsigned long)durationMs) {
            return true;
        }
        //add a small delay to avoid busy-waiting
        delay(1);
    }
    return false;
}