#include <Arduino.h>

#include "actuators/MecanumDrive.h"
#include "comms/UART.h"
#include "core/StateMachine.h"
#include "core/WifiManager.h"

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

    UART::Data sensorData =UART::getData();

    if (sensorData.valid) {
        StateMachine::update(
            sensorData.mag1,
            sensorData.mag2
        );
    } else {
        StateMachine::update(0, 0);
    }
    delay(5);
}