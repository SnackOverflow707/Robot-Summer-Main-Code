/*
 * Correction strategy:
 *   Small error  → strafe sideways (straight sections)
 *   Large error  → rotate (curved sections)
 */

#include "TapeFollower.h"
#include "actuators/MecanumDrive.h"

// UPDATE!
#define LEFT_SENSOR_PIN 12
#define RIGHT_SENSOR_PIN 13

/*
// Constants
#define WHITE_THRESHOLD 1.65f   // volts
#define MAX_ADC_VALUE 8191    // ESP32-S3, 13-bit ADC
*/

#define BASE_SPEED        150
#define STRAFE_SPEED       80
#define ROTATE_SPEED      100
#define CURVE_THRESHOLD     5

float Kp = 40.0f;
float Ki =  0.0f;
float Kd = 15.0f;

float integral  = 0.0f;
float lastError = 0.0f;

static int  largeErrorCount = 0;   // consecutive large error readings
static bool curveMode = false;
// add this near the top with your other static variables
static enum { GOING_FORWARD, STRAFING, ROTATING } lastMode = GOING_FORWARD;


extern MecanumDrive drive;

// Forward drive
//float readSensorVoltage(int pin);
float getError(bool leftWhite, bool rightWhite);
float computePID(float error);
void  applyCorrection(float error);

/*
float readSensorVoltage(int pin) {
    int raw = analogRead(pin);
    return raw * (3.3f / MAX_ADC_VALUE);
}
*/

// Mapping sensor states to error value (-1, 0, 1)
float getError(bool leftWhite, bool rightWhite) {
    if (!leftWhite && !rightWhite) return 0.0f;   // both on black (centered)
    if (!leftWhite &&  rightWhite) return  1.0f;  // right sensor is white (line drifted left)
    if ( leftWhite && !rightWhite) return -1.0f;  // left sensor is white (line drifted right)
    return lastError;                              // both sensors are white (LOST)
}

// Computing PID
float computePID(float error) {
    integral += error;
    float derivative = error - lastError;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;
    output = constrain(output, -255.0f, 255.0f);
    return output;
}


void applyCorrection(float error) {
    if (error == 0.0f) {
        if (lastMode != GOING_FORWARD) {
            delay(10);   // transition to forward
        }
        largeErrorCount = 0;
        curveMode = false;
        drive.forward(BASE_SPEED);
        lastMode = GOING_FORWARD;
        return;
    }

    float correction = computePID(error);
    int correctionSpeed = (int)abs(correction);
    correctionSpeed = constrain(correctionSpeed, 0, 255);

    largeErrorCount++;
    if (largeErrorCount >= CURVE_THRESHOLD) curveMode = true;

    if (!curveMode) {
        int speed = constrain(correctionSpeed, 0, STRAFE_SPEED);
        if (lastMode != STRAFING) {
            delay(10);   // transition to strafe
        }
        drive.forward(BASE_SPEED);
        if (error > 0) drive.strafeLeft(speed);
        else           drive.strafeRight(speed);
        lastMode = STRAFING;
    } else {
        int speed = constrain(correctionSpeed, 0, ROTATE_SPEED);
        if (lastMode != ROTATING) {
            delay(10);   // transition to rotate
        }
        drive.forward(BASE_SPEED);
        if (error > 0) drive.rotateCounterClockwise(speed);
        else           drive.rotateClockwise(speed);
        lastMode = ROTATING;
    }
}

void tapeFollowStep() { 
    /*
    bool leftWhite = readSensorVoltage(LEFT_SENSOR_PIN)  > WHITE_THRESHOLD;
    bool rightWhite = readSensorVoltage(RIGHT_SENSOR_PIN) > WHITE_THRESHOLD;
    */
   
    bool leftWhite = digitalRead(LEFT_SENSOR_PIN) == HIGH;
    bool rightWhite = digitalRead(RIGHT_SENSOR_PIN) == HIGH;

    float error = getError(leftWhite, rightWhite);
    applyCorrection(error);
    
} 