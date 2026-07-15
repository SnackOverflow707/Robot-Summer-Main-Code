/*
 * Correction strategy:
 *   Only rotate
 */
#include "TapeFollower.h"
#include "actuators/MecanumDrive.h"

// UPDATE!
#define LEFT_SENSOR_PIN 13
#define RIGHT_SENSOR_PIN 12

// Constants
#define WHITE_THRESHOLD 1.7f   // Using 3.3K pull up, 100 LED
#define MAX_ADC_VALUE 8191    // ESP32-S3, 13-bit ADC

#define BASE_SPEED        60
#define ROTATE_SPEED      30

float Kp = 20.0f;
float Ki =  0.0f;
float Kd = 5.0f;

float integral  = 0.0f;
float lastError = 0.0f;
float pidLastError = 0.0f;

extern MecanumDrive drive;

// Forward drive
float readSensorVoltage(int pin);
float getError(bool leftWhite, bool rightWhite);
float computePID(float error);
void  applyCorrection(float error);

float readSensorVoltage(int pin) {
    int raw = analogRead(pin);
    return raw * (3.3f / MAX_ADC_VALUE);
}

// Mapping sensor states to error value (-1, 0, 1)
float getError(bool leftWhite, bool rightWhite) {
    float error;
    if (!leftWhite && !rightWhite) {
        error = 0.0f;
    } else if (!leftWhite && rightWhite) {
        error = 1.0f;
    } else if (leftWhite && !rightWhite) {
        error = -1.0f;
    } else {
        return lastError;   // both white — return without updating lastError
    }
    lastError = error;      // only update when we have a real reading
    return error;
}

// Computing PID
float computePID(float error) {
    integral += error;
    float derivative = error - pidLastError;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    pidLastError = error;
    output = constrain(output, -255.0f, 255.0f);
    return output;
}

void applyCorrection(float error) {
    if (error == 0.0f) {
        drive.forward(BASE_SPEED);
        return;
    }

    float correction = computePID(error);
    int speed = constrain((int)abs(correction), 0, ROTATE_SPEED);

    drive.forward(BASE_SPEED);
    if (error > 0) drive.rotateCounterClockwiseBackAxis(speed);
    else           drive.rotateClockwiseBackAxis(speed);
}

void tapeFollowStep() { 

    bool leftWhite = readSensorVoltage(LEFT_SENSOR_PIN)  < WHITE_THRESHOLD;
    bool rightWhite = readSensorVoltage(RIGHT_SENSOR_PIN) < WHITE_THRESHOLD;

    float error = getError(leftWhite, rightWhite);
    applyCorrection(error);
    
} 