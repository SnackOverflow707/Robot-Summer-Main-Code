#include <Arduino.h>
#include "actuators/MecanumDrive.h"
#include "tape_logic/TapeFollower.h"

MecanumDrive drive;

void setup() {
    Serial.begin(115200);
    pinMode(12, INPUT);   // LEFT_SENSOR_PIN
    pinMode(13, INPUT);   // RIGHT_SENSOR_PIN
    drive.begin();
}

void loop() {
    tapeFollowStep();
}