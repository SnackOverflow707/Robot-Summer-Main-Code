/*
Read sensor voltage - get thresholds tomorrow
When below the threshold, we are on the side tape

Stop the robot and call the arm functions - calibration when arm is mounted on the chassis

grab the first tower piece then grip the tower funnel to the tower stand, then grab the other pieces??

Off tape when voltage high, on tape when voltage low
*/

#include "SideSensors.h"
#include "actuators/MecanumDrive.h"

#define SENSOR_PIN 10 // UPDATE!
#define MAX_ADC_VALUE 8191
#define WHITE_THRESHOLD 1.0f // UPDATE!

extern MecanumDrive drive;

// Wifi variables
static float latestSensorVoltage = 0.0f;
static bool latestOnTape = false;

float readSideSensorVoltage(int pin);
bool sensorTriggered(float rawSensorVal);

float readSideSensorVoltage(int pin){
    int raw = analogRead(pin);
    return raw * (3.3f / MAX_ADC_VALUE);
}

bool sensorTriggered(float voltage) {
    return voltage > WHITE_THRESHOLD;
}

void checkForSideTape() {
    latestSensorVoltage = readSideSensorVoltage(SENSOR_PIN);  
    latestOnTape = sensorTriggered(latestSensorVoltage);  

    if (latestOnTape) {
        drive.stop();
        // TODO: arm functions
        delay(3000);
    }
}

SideSensorStatus getSideSensorStatus() {
    SideSensorStatus status;
    status.sensorVoltage = latestSensorVoltage;
    status.onTape = latestOnTape;
    return status;
}




