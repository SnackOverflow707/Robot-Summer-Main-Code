/*
 * Correction strategy:
 *   Only rotate
 */
#include "TapeFollower.h"
#include "actuators/MecanumDrive.h"

// UPDATE!
#define LEFT_SENSOR_PIN 14
#define RIGHT_SENSOR_PIN 13

// Constants
#define LEFT_WHITE_THRESHOLD 0.5f   // Using 3.3K pull up, 100 LED
#define RIGHT_WHITE_THRESHOLD 1.0F

#define MAX_ADC_VALUE 8191    // ESP32-S3, 13-bit ADC

static int baseSpeed = 120;
#define ROTATE_SPEED      60
#define SEARCH_ROTATE_SPEED 140

// +1 or -1, representing the most recent commanded turn direction
static int lastTurnDirection = 1;
static bool searchingForTape = false;

float Kp = 150.0f;
float Ki =  0.0f;
float Kd = 120.0f;

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

void applyCorrection(float error)
{
    float correction = computePID(error);

    int rotateSpeed = constrain(
        (int)correction,
        -min(ROTATE_SPEED, baseSpeed),
         min(ROTATE_SPEED, baseSpeed)
    );

    // Your robot currently needs the PID direction reversed
    int rotateCommand = -rotateSpeed;

    if (rotateCommand > 0) {
        lastTurnDirection = 1;
    }
    else if (rotateCommand < 0) {
        lastTurnDirection = -1;
    }

    drive.forwardWithRotate(baseSpeed, rotateCommand);
}

void tapeFollowStep()
{
    if (!tapeFollowingEnabled) return;

    latestLeftVoltage  = readSensorVoltage(LEFT_SENSOR_PIN);
    latestRightVoltage = readSensorVoltage(RIGHT_SENSOR_PIN);

    latestLeftWhite =
        latestLeftVoltage < LEFT_WHITE_THRESHOLD;

    latestRightWhite =
        latestRightVoltage < RIGHT_WHITE_THRESHOLD;

    bool bothOffTape =
        latestLeftWhite && latestRightWhite;

    if (bothOffTape)
    {
        delay (20);
        searchingForTape = true;

        // Rotate in the same direction as the most recent correction
        drive.rotateAboutCenter(
            lastTurnDirection * SEARCH_ROTATE_SPEED
        );

        return;
    }

    // At least one sensor has found the tape
    searchingForTape = false;

    latestError = getError(
        latestLeftWhite,
        latestRightWhite
    );

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
//right sensor was on tape but moved left
void updateTapeSensors()
{
    latestLeftVoltage = readSensorVoltage(LEFT_SENSOR_PIN);
    latestRightVoltage = readSensorVoltage(RIGHT_SENSOR_PIN);

    latestLeftWhite = latestLeftVoltage < LEFT_WHITE_THRESHOLD;
    latestRightWhite = latestRightVoltage < RIGHT_WHITE_THRESHOLD;
}
void setBaseSpeed(int speed)
{
    baseSpeed = constrain(speed, 0, 255);
}

int getBaseSpeed()
{
    return baseSpeed;
}