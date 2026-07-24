#include <Arduino.h>

#include "actuators/MecanumDrive.h"
#include "comms/UART.h"
#include "core/StateMachine.h"
#include "core/WifiManager.h"
#include "robotArm/ArmController2.h"
#include "tape_logic/TapeFollower.h"
#include "tape_logic/SideSensors.h"

MecanumDrive drive;
//ArmController2 arm;
ArmController2 arm;

// The SSID/password arguments are unused in access-point mode.
WifiManager wifi("", "", drive, arm);

void setup()
{
    Serial.begin(115200);

    drive.begin();
    arm.begin();
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

    updateTapeSensors();
    checkForSideTape();

    const UART::Data& sensorData = UART::getData();
    const TapeFollowerStatus tapeStatus = getTapeFollowerStatus();
    const SideSensorStatus sideStatus = getSideSensorStatus();
    StateMachine::Inputs inputs;

    /*bool wifiRequested = (sensorData.mask & 0x04) != 0;
    if (wifiRequested){*/
        wifi.update();
    //}
    const UART::MetalData metal0 = UART::getMetalData(0);
    const UART::MetalData metal1 = UART::getMetalData(1);
    
    inputs.metalMagnitude0 =
        metal0.valid
            ? static_cast<uint16_t>(
                constrain(
                    metal0.frequencyHz,
                    0.0f,
                    65535.0f))
            : 0;
    
    inputs.metalMagnitude1 =
        metal1.valid
            ? static_cast<uint16_t>(
                constrain(
                    metal1.frequencyHz,
                    0.0f,
                    65535.0f))
            : 0;

    inputs.mag1 = sensorData.valid ? sensorData.mag1 : 0;
    inputs.mag2 = sensorData.valid ? sensorData.mag2 : 0;

    inputs.sideTapeDetected = checkForSideTape();
    inputs.returnTapeDetected = false;

    StateMachine::update(inputs);
    delay(5);
    
}