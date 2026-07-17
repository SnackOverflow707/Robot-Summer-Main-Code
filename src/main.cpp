#include <Arduino.h>

#include "actuators/MecanumDrive.h"
#include "tape_logic/TapeFollower.h"
#include "tape_logic/SideSensors.h"
#include "core/WifiManager.h"

MecanumDrive drive;

// The SSID/password arguments are unused in access-point mode.
WifiManager wifi("", "", drive);

void setup()
{
    Serial.begin(115200);

    pinMode(12, INPUT);
    pinMode(13, INPUT);

    drive.begin();

    wifi.begin();
    wifi.enable();
}

void loop()
{
    wifi.update();
    checkForSideTape();
    tapeFollowStep();
    updateTapeSensors();    

    delay(5);
}