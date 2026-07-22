#include <Arduino.h>

#include "actuators/MecanumDrive.h"
#include "comms/UART.h"
#include "core/StateMachine.h"
#include "core/WifiManager.h"
#include "tape_logic/TapeFollower.h"
#include "tape_logic/SideSensors.h"

MecanumDrive drive;

// The SSID/password arguments are unused in access-point mode.
WifiManager wifi("", "", drive);

void setup()
{
    Serial.begin(115200);

    drive.begin();
    UART::begin();

    StateMachine::begin();

    pinMode(14, INPUT);
    pinMode(13, INPUT);

    wifi.begin();
    wifi.enable();
}

void loop()
{
    UART::update();
    wifi.update();

    updateTapeSensors();
    checkForSideTape();

    const UART::Data& sensorData = UART::getData();
    const TapeFollowerStatus tapeStatus = getTapeFollowerStatus();
    const SideSensorStatus sideStatus = getSideSensorStatus();

    StateMachine::Inputs inputs;

    inputs.mag1 = sensorData.valid ? sensorData.mag1 : 0;
    inputs.mag2 = sensorData.valid ? sensorData.mag2 : 0;

    inputs.metalMagnitude = 0;

    inputs.sideTapeDetected = checkForSideTape();
    inputs.returnTapeDetected = false;

    StateMachine::update(inputs);
    delay(5);
    
}