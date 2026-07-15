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
#define LEFT_WHITE_THRESHOLD 0.5f   // Using 3.3K pull up, 100 LED
#define RIGHT_WHITE_THRESHOLD 1.0F

#define MAX_ADC_VALUE 8191    // ESP32-S3, 13-bit ADC

#define BASE_SPEED        120
#define ROTATE_SPEED      60

float Kp = 20.0f;
float Ki =  0.0f;
float Kd = 5.0f;

float integral  = 0.0f;
float lastError = 0.0f;
float pidLastError = 0.0f;

static bool tapeFollowingEnabled = false;

//Wifi Variables
static float latestLeftVoltage = 0.0f;
static float latestRightVoltage = 0.0f;

static bool latestLeftWhite = false;
static bool latestRightWhite = false;

static float latestError = 0.0f;
static float latestPIDOutput = 0.0f;
static float latestDerivative = 0.0f;

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
    latestDerivative = error - pidLastError;

    float output =
        (Kp * error) +
        (Ki * integral) +
        (Kd * latestDerivative);

    pidLastError = error;
    output = constrain(output, -255.0f, 255.0f);
    latestPIDOutput = output;
    return output;
}

// void applyCorrection(float error) {
//     if (error == 0.0f) {
//         drive.forward(BASE_SPEED);
//         return;
//     }

//     float correction = computePID(error);
//     int speed = constrain((int)abs(correction), 0, ROTATE_SPEED);

//     drive.forward(BASE_SPEED);
//     if (error > 0) drive.rotateCounterClockwiseBackAxis(speed);
//     else           drive.rotateClockwiseBackAxis(speed);
// }

// static enum { FORWARD, ROTATING_CCW, ROTATING_CW } motorState = FORWARD;

void applyCorrection(float error) {
    if (error == 0.0f) {
        drive.forward(BASE_SPEED);
        return;
    }

    float correction = computePID(error);
    //int speed = constrain((int)abs(correction), 0, ROTATE_SPEED); possible fix here
    int speed = constrain((int)abs(correction), 0, min(ROTATE_SPEED, BASE_SPEED));

    // error > 0 = drifted left = rotate clockwise = positive rotSpeed
    // error < 0 = drifted right = rotate counter-clockwise = negative rotSpeed
    if (error > 0) drive.forwardWithRotateBackAxis(BASE_SPEED,  speed);
    else           drive.forwardWithRotateBackAxis(BASE_SPEED, -speed);
}

void tapeFollowStep() { 
    if (!tapeFollowingEnabled) return;

    latestLeftVoltage  = readSensorVoltage(LEFT_SENSOR_PIN);
    latestRightVoltage = readSensorVoltage(RIGHT_SENSOR_PIN);

    latestLeftWhite  = latestLeftVoltage  > LEFT_WHITE_THRESHOLD;
    latestRightWhite = latestRightVoltage > RIGHT_WHITE_THRESHOLD;

    latestError = getError(latestLeftWhite, latestRightWhite);
    applyCorrection(latestError);
}


//Wifi Functions
TapeFollowerStatus getTapeFollowerStatus()
{
    TapeFollowerStatus status;

    status.leftVoltage = latestLeftVoltage;
    status.rightVoltage = latestRightVoltage;

    status.leftWhite = latestLeftWhite;
    status.rightWhite = latestRightWhite;

    status.error = latestError;
    status.pidOutput = latestPIDOutput;
    status.integral = integral;
    status.derivative = latestDerivative;

    status.kp = Kp;
    status.ki = Ki;
    status.kd = Kd;

    return status;
}

void setTapePID(float kp, float ki, float kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;

    resetTapePID();
}

void resetTapePID()
{
    integral = 0.0f;
    lastError = 0.0f;
    pidLastError = 0.0f;

    latestError = 0.0f;
    latestPIDOutput = 0.0f;
    latestDerivative = 0.0f;
}

bool isTapeFollowingEnabled()
{
    return tapeFollowingEnabled;
}

void setTapeFollowing(bool enabled)
{
    tapeFollowingEnabled = enabled;

    if (!enabled)
    {
        drive.stop();
    }
}
void updateTapeSensors()
{
    latestLeftVoltage = readSensorVoltage(LEFT_SENSOR_PIN);
    latestRightVoltage = readSensorVoltage(RIGHT_SENSOR_PIN);

    latestLeftWhite = latestLeftVoltage < LEFT_WHITE_THRESHOLD;
    latestRightWhite = latestRightVoltage < RIGHT_WHITE_THRESHOLD;
}