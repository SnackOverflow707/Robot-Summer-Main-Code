#include <Arduino.h>

#include "actuators/MecanumDrive.h"
#include "comms/UART.h"
#include "core/StateMachine.h"
#include "core/WifiManager.h"
#include "robotArm/ArmController2.h"
#include "tape_logic/TapeFollower.h"
#include "tape_logic/SideSensors.h"
#include "robotArm/taskManager.h"
#include "core/states/TowerRam.h"

MecanumDrive drive;
//ArmController2 arm;
ArmController2 arm;
TaskManager taskManager(arm);
// The SSID/password arguments are unused in access-point mode.
WifiManager wifi("", "", drive, arm);

void setup()
{
    Serial.begin(115200);

    drive.begin();
    arm.begin();
    UART::begin();
    TowerRam::begin();
    pinMode(14, INPUT);
    pinMode(13, INPUT);

    wifi.begin();
    wifi.enable();
    
    delay(1000);

    StateMachine::begin();
    StateMachine::setEnabled(true);
}

void loop()
{
    UART::update();

    updateTapeSensors();
    checkForSideTape();

    const UART::Data sensorData = UART::getData();
    const UART::PoseData poseData = UART::getPoseData();

    const TapeFollowerStatus tapeStatus =
        getTapeFollowerStatus();

    const SideSensorStatus sideStatus =
        getSideSensorStatus();

    StateMachine::Inputs inputs;

    wifi.update();

    const UART::MetalData metal0 =
        UART::getMetalData(0);

    const UART::MetalData metal1 =
        UART::getMetalData(1);


        inputs.metalMagnitude0 =
        metal0.valid
            ? metal0.frequencyHz
            : 0.0f;
    
    inputs.metalMagnitude1 =
        metal1.valid
            ? metal1.frequencyHz
            : 0.0f;


    inputs.mag1 =
        sensorData.valid
            ? sensorData.mag1
            : 0;

    inputs.mag2 =
        sensorData.valid
            ? sensorData.mag2
            : 0;


    inputs.sideTapeDetected =
        sideStatus.onTape;

    inputs.returnTapeDetected = false;


    // Pose data is now available here
    if (poseData.valid)
    {
        // poseData.x
        // poseData.y
        // poseData.theta
        // poseData.vx
        // poseData.vy
        // poseData.omega
    }


    StateMachine::update(inputs);

    delay(5);
}