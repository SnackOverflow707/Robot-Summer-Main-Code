#include "actuators/MecanumDrive.h"
#include "config/pins.h"
#include "comms/UART.h"

#include <Arduino.h>
#include <math.h>

namespace
{
// PoseData x/y are meters, theta is radians (world frame).
constexpr float DRIVE_TO_TOLERANCE_M = 0.02f;
constexpr float DRIVE_TO_SLOWDOWN_RADIUS_M = 0.15f;
constexpr unsigned long DRIVE_TO_TIMEOUT_MS = 4000;
} // namespace

MecanumDrive::MecanumDrive()
    : frontLeft(
          FL_A,
          FL_B,
          MCPWM_UNIT_0,
          MCPWM_TIMER_0
      ),
      frontRight(
          FR_A,
          FR_B,
          MCPWM_UNIT_0,
          MCPWM_TIMER_1
      ),
      backLeft(
          BL_A,
          BL_B,
          MCPWM_UNIT_0,
          MCPWM_TIMER_2
      ),
      backRight(
          BR_A,
          BR_B,
          MCPWM_UNIT_1,
          MCPWM_TIMER_0
      )
{
}

void MecanumDrive::begin()
{
    frontLeft.begin();
    frontRight.begin();
    backLeft.begin();
    backRight.begin();
}

void MecanumDrive::forward(int speed)
{
    frontLeft.setSpeed(speed);
    frontRight.setSpeed(speed);
    backLeft.setSpeed(speed);
    backRight.setSpeed(speed);
}

void MecanumDrive::backward(int speed)
{
    forward(-speed);
}

void MecanumDrive::strafeRight(int speed)
{
    frontLeft.setSpeed(speed);
    frontRight.setSpeed(-speed);
    backLeft.setSpeed(-speed);
    backRight.setSpeed(speed);
}

void MecanumDrive::strafeLeft(int speed)
{
    strafeRight(-speed);
}

void MecanumDrive::rotateClockwise(int speed)
{
    frontLeft.setSpeed(speed);
    frontRight.setSpeed(-speed);
    backLeft.setSpeed(speed);
    backRight.setSpeed(-speed);
}

void MecanumDrive::rotateCounterClockwise(int speed)
{
    rotateClockwise(-speed);
}

void MecanumDrive::rotateClockwiseBackAxis(int speed)
{
    frontLeft.setSpeed(speed);
    frontRight.setSpeed(-speed);
    backLeft.stop();
    backRight.stop();
}

void MecanumDrive::rotateCounterClockwiseBackAxis(int speed)
{
    rotateClockwiseBackAxis(-speed);
}

void MecanumDrive::stop()
{
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}

void MecanumDrive::frontLeftMotor(int speed)
{
    frontLeft.setSpeed(speed);
}

void MecanumDrive::frontRightMotor(int speed)
{
    frontRight.setSpeed(speed);
}

void MecanumDrive::backLeftMotor(int speed)
{
    backLeft.setSpeed(speed);
}

void MecanumDrive::backRightMotor(int speed)
{
    backRight.setSpeed(speed);
}

void MecanumDrive::forwardWithRotateBackAxis(int fwdSpeed, int rotSpeed)
{
    backLeft.setSpeed(fwdSpeed);
    backRight.setSpeed(fwdSpeed);

    frontLeft.setSpeed(fwdSpeed + rotSpeed);
    frontRight.setSpeed(fwdSpeed - rotSpeed);
}

void MecanumDrive::forwardWithRotate(int forwardSpeed, int rotateSpeed)
{
    int leftSpeed =
        constrain(forwardSpeed + rotateSpeed, 0, 255);

    int rightSpeed =
        constrain(forwardSpeed - rotateSpeed, 0, 255);

    frontLeft.setSpeed(leftSpeed);
    backLeft.setSpeed(leftSpeed);

    frontRight.setSpeed(rightSpeed);
    backRight.setSpeed(rightSpeed);
}

void MecanumDrive::rotateAboutCenter(int rotateSpeed)
{
    rotateSpeed = constrain(
        rotateSpeed,
        -255,
        255
    );

    frontLeft.setSpeed(rotateSpeed);
    backLeft.setSpeed(rotateSpeed);

    frontRight.setSpeed(-rotateSpeed);
    backRight.setSpeed(-rotateSpeed);
}

void MecanumDrive::leftWheelsForward(int speed)
{
    delay(10);

    frontLeft.setSpeed(speed);
    backLeft.setSpeed(speed);

    frontRight.setSpeed(0);
    backRight.setSpeed(0);
}

// NOTE: these two are still blocking -- they do not return until the
// move finishes or bails out, during which nothing else on this board
// runs. IRAlignerManual no longer calls these for exactly that reason
// (see IRAlignerManual.cpp, which now drives incrementally from its own
// non-blocking update() using UART::getPoseData() directly). Kept here,
// bugs fixed, in case something else still wants a simple blocking
// "drive until this far" call.
//
// FIXES applied vs. the original:
//   1. Variable shadowing bug: the original declared `flowData` once
//      before the loop, then declared a SECOND, shadowed `flowData`
//      inside the loop body (only used for the position read). The
//      `.valid` check was reading the stale, function-entry snapshot
//      every single iteration, never the live value. Now there's one
//      `flowData`, reassigned fresh at the top of every iteration.
//   2. `stop()` was called on hitting maxInvalidReadings, then execution
//      fell straight through into another backward()/strafeRight() call
//      in the same iteration, undoing the stop. Now it `break`s out of
//      the loop immediately instead.
//   3. Default argument moved to the header declaration only -- having
//      `= 10` here too is a compile error if MecanumDrive.h also
//      declares this function (redefinition of default argument).
//   4. Negative `distance` used to silently do nothing (the loop
//      condition `abs(current-start) <= distance` was false from the
//      first check). Now the sign of `distance` picks the direction.

void MecanumDrive::driveBackward(float distance, int speed, int maxInvalidReadings)
{
    UART::PoseData flowData = UART::getPoseData();
    const float startY = flowData.y;
    float currentY = startY;
    int currentInvalidReadings = 0;

    const bool driveForward = (distance < 0.0f);
    const float targetDistance = fabsf(distance);

    while (fabsf(currentY - startY) <= targetDistance &&
           currentInvalidReadings < maxInvalidReadings)
    {
        flowData = UART::getPoseData();

        if (!flowData.valid)
        {
            ++currentInvalidReadings;
        }
        else
        {
            currentInvalidReadings = 0;
            currentY = flowData.y;
        }

        if (currentInvalidReadings >= maxInvalidReadings)
        {
            break;
        }

        if (driveForward)
        {
            forward(speed);
        }
        else
        {
            backward(speed);
        }
    }

    stop();
}

void MecanumDrive::strafeRightWithDist(float distance, int speed, int maxInvalidReadings)
{
    UART::PoseData flowData = UART::getPoseData();
    const float startX = flowData.x;
    float currentX = startX;
    int currentInvalidReadings = 0;

    const bool driveLeft = (distance < 0.0f);
    const float targetDistance = fabsf(distance);

    while (fabsf(currentX - startX) <= targetDistance &&
           currentInvalidReadings < maxInvalidReadings)
    {
        flowData = UART::getPoseData();

        if (!flowData.valid)
        {
            ++currentInvalidReadings;
        }
        else
        {
            currentInvalidReadings = 0;
            currentX = flowData.x;
        }

        if (currentInvalidReadings >= maxInvalidReadings)
        {
            break;
        }

        if (driveLeft)
        {
            strafeLeft(speed);
        }
        else
        {
            strafeRight(speed);
        }
    }

    stop();
}