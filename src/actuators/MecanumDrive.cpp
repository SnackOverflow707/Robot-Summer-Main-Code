#include "actuators/MecanumDrive.h"
#include "config/pins.h"
#include "comms/UART.h"

#include <Arduino.h>

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

void MecanumDrive::driveTo(
    float dx,
    float dy,
    int speed
)
{
    stop();

    // Get the starting pose.
    UART::update();

    const UART::PoseData start =
        UART::getPoseData();

    if (!start.valid)
    {
        return;
    }

    // dx/dy are offsets from the current world position.
    const float targetX =
        start.x + dx;

    const float targetY =
        start.y + dy;

    const unsigned long startTime =
        millis();

    while (true)
    {
        UART::update();

        const UART::PoseData pose =
            UART::getPoseData();

        // Lost pose data.
        if (!pose.valid)
        {
            break;
        }

        const float errorX =
            targetX - pose.x;

        const float errorY =
            targetY - pose.y;

        const float distance =
            sqrtf(
                errorX * errorX +
                errorY * errorY
            );

        // Target reached.
        if (distance <= DRIVE_TO_TOLERANCE_M)
        {
            break;
        }

        // Safety timeout.
        if (
            millis() - startTime >=
            DRIVE_TO_TIMEOUT_MS
        )
        {
            break;
        }

        // Convert world-frame error into robot-frame error.
        const float cosTheta =
            cosf(pose.theta);

        const float sinTheta =
            sinf(pose.theta);

        const float forwardError =
            errorX * cosTheta +
            errorY * sinTheta;

        const float strafeError =
            -errorX * sinTheta +
            errorY * cosTheta;

        // Slow down near the destination.
        const float speedScale =
            min(
                1.0f,
                distance /
                    DRIVE_TO_SLOWDOWN_RADIUS_M
            );

        const float commandMagnitude =
            speed * speedScale;

        const int forwardSpeed =
            static_cast<int>(
                commandMagnitude *
                (forwardError / distance)
            );

        const int strafeSpeed =
            static_cast<int>(
                commandMagnitude *
                (strafeError / distance)
            );

        // Mecanum mixing.
        frontLeft.setSpeed(
            constrain(
                forwardSpeed + strafeSpeed,
                -255,
                255
            )
        );

        frontRight.setSpeed(
            constrain(
                forwardSpeed - strafeSpeed,
                -255,
                255
            )
        );

        backLeft.setSpeed(
            constrain(
                forwardSpeed - strafeSpeed,
                -255,
                255
            )
        );

        backRight.setSpeed(
            constrain(
                forwardSpeed + strafeSpeed,
                -255,
                255
            )
        );

        delay(5);
    }

    stop();
}