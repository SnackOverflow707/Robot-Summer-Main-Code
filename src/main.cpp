#include <Arduino.h>
#include "actuators/MecanumDrive.h"

MecanumDrive drive;

void setup() {
    drive.begin();
}

void loop() {
    // Forward
    drive.forward(150);
    delay(1000);
}