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

extern int BASE_SPEED;    // forward drive speed 
extern int STRAFE_SPEED;    // sideways correction speed
extern int ROTATE_SPEED;  // rotation correction speed

// how many consecutive large-error readings before switching to rotate mode (meaning we have encountered a curve along the path)
extern int CURVE_THRESHOLD;

// PID for strafe correction
extern float Kp;
extern float Ki;
extern float Kd;

float integral  = 0.0f;
float lastError = 0.0f;

static int  largeErrorCount = 0;   // consecutive large error readings
static bool curveMode = false;

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

// 
void applyCorrection(float error) {
    if (error == 0.0f) {
        // centred — drive straight and reset curve counter
        largeErrorCount = 0;
        curveMode = false;
        drive.forward(BASE_SPEED);
        return;
    }

    // track how long we've had a large error
    largeErrorCount++;
    if (largeErrorCount >= CURVE_THRESHOLD) {
        curveMode = true; // no longer strafing
    }

    if (!curveMode) {
        // Strafing mode
        drive.forward(BASE_SPEED);
        if (error > 0) { 
            // strafe left
            drive.strafeLeft(STRAFE_SPEED);
        } else {
            // strafe right
            drive.strafeRight(STRAFE_SPEED);
        }
    } else {
        // Curving mode
        drive.forward(BASE_SPEED);
        if (error > 0) {
            // rotate left (ccw)
            drive.rotateCounterClockwise(ROTATE_SPEED);
        } else {
            // rotate right (cw)
            drive.rotateClockwise(ROTATE_SPEED);
        }
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