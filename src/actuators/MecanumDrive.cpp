#include "actuators/MecanumDrive.h"
#include "config/pins.h"
#include <Arduino.h>

#include "actuators/MecanumDrive.h"
#include "config/pins.h"
#include "comms/UART.h"

namespace
{
// FlowPose x/y are meters, theta is radians (world frame) - see
// Sensor_ESP_Arduino/src/Flowsensor/Flowsensor.cpp.
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

void MecanumDrive::begin() {
  frontLeft.begin();
  frontRight.begin();
  backLeft.begin();
  backRight.begin();
}

void MecanumDrive::forward(int speed) {
  frontLeft.setSpeed(speed);
  frontRight.setSpeed(speed);
  backLeft.setSpeed(speed);
  backRight.setSpeed(speed);
}

void MecanumDrive::backward(int speed) {
  forward(-speed);
}

void MecanumDrive::strafeRight(int speed) {
  frontLeft.setSpeed(speed);
  frontRight.setSpeed(-speed);
  backLeft.setSpeed(-speed);
  backRight.setSpeed(speed);
}

void MecanumDrive::strafeLeft(int speed) {
  strafeRight(-speed);
}

void MecanumDrive::rotateClockwise(int speed) {
  frontLeft.setSpeed(speed);
  frontRight.setSpeed(-speed);
  backLeft.setSpeed(speed);
  backRight.setSpeed(-speed);
}

void MecanumDrive::rotateCounterClockwise(int speed) {
  rotateClockwise(-speed);
}

void MecanumDrive::rotateClockwiseBackAxis(int speed) {
  frontLeft.setSpeed(speed);
  frontRight.setSpeed(-speed);
  backLeft.stop();
  backRight.stop();
}

void MecanumDrive::rotateCounterClockwiseBackAxis(int speed) {
  rotateClockwiseBackAxis(-speed);
}

void MecanumDrive::stop() {
  frontLeft.stop();
  frontRight.stop();
  backLeft.stop();
  backRight.stop();
}

void MecanumDrive::frontLeftMotor(int speed) {
  frontLeft.setSpeed(speed);
}

void MecanumDrive::frontRightMotor(int speed) {
  frontRight.setSpeed(speed);
}

void MecanumDrive::backLeftMotor(int speed) {
  backLeft.setSpeed(speed);
}

void MecanumDrive::backRightMotor(int speed) {
  backRight.setSpeed(speed);
}

void MecanumDrive::forwardWithRotateBackAxis(int fwdSpeed, int rotSpeed) {
    // back wheels drive straight forward
    backLeft.setSpeed(fwdSpeed);
    backRight.setSpeed(fwdSpeed);

    // front wheels: forward + rotation
    // rotSpeed > 0 = clockwise, rotSpeed < 0 = counter-clockwise
    frontLeft.setSpeed(fwdSpeed + rotSpeed);
    frontRight.setSpeed(fwdSpeed - rotSpeed);
}

void MecanumDrive::forwardWithRotate(int forwardSpeed, int rotateSpeed)
{
    int leftSpeed  = constrain(forwardSpeed + rotateSpeed, 0, 255);
    int rightSpeed = constrain(forwardSpeed - rotateSpeed, 0, 255);

    frontLeft.setSpeed(leftSpeed);
    backLeft.setSpeed(leftSpeed);

    frontRight.setSpeed(rightSpeed);
    backRight.setSpeed(rightSpeed);
}
void MecanumDrive::rotateAboutCenter(int rotateSpeed)
{
    rotateSpeed = constrain(rotateSpeed, -255, 255);

    frontLeft.setSpeed( rotateSpeed);
    backLeft.setSpeed(  rotateSpeed);

    frontRight.setSpeed(-rotateSpeed);
    backRight.setSpeed( -rotateSpeed);
}
void MecanumDrive::leftWheelsForward(int speed)
{
  delay(10);
    frontLeft.setSpeed(speed);
    backLeft.setSpeed(speed);

    frontRight.setSpeed(0);
    backRight.setSpeed(0);
}

void MecanumDrive::driveTo(float dx, float dy, int speed) {
  UART::update();
  const UART::FlowPose start = UART::getFlowPose();

  if (!start.valid) {
    stop();
    return;
  }

  const float targetX = start.x + dx;
  const float targetY = start.y + dy;

  const unsigned long startTime = millis();

  while (true) {
    UART::update();
    const UART::FlowPose pose = UART::getFlowPose();

    // Lost tracking mid-move - safer to stop than to keep driving blind.
    if (!pose.valid) {
      break;
    }

    const float errorX = targetX - pose.x;
    const float errorY = targetY - pose.y;
    const float distance = sqrtf(errorX * errorX + errorY * errorY);

    if (distance <= DRIVE_TO_TOLERANCE_M) {
      break;
    }

    if (millis() - startTime >= DRIVE_TO_TIMEOUT_MS) {
      break;
    }

    // Rotate the world-frame error into the robot's body frame so we can
    // command forward/strafe directly - a mecanum base doesn't need to
    // turn to face the target the way a differential drive would.
    const float cosTheta = cosf(pose.theta);
    const float sinTheta = sinf(pose.theta);
    const float forwardError = errorX * cosTheta + errorY * sinTheta;
    const float strafeError = -errorX * sinTheta + errorY * cosTheta;

    // Ramp speed down as we approach the target to avoid overshoot.
    const float speedScale = min(1.0f, distance / DRIVE_TO_SLOWDOWN_RADIUS_M);
    const float commandMagnitude = speed * speedScale;

    const int forwardSpeed = static_cast<int>(commandMagnitude * (forwardError / distance));
    const int strafeSpeed = static_cast<int>(commandMagnitude * (strafeError / distance));

    frontLeft.setSpeed(constrain(forwardSpeed + strafeSpeed, -255, 255));
    frontRight.setSpeed(constrain(forwardSpeed - strafeSpeed, -255, 255));
    backLeft.setSpeed(constrain(forwardSpeed - strafeSpeed, -255, 255));
    backRight.setSpeed(constrain(forwardSpeed + strafeSpeed, -255, 255));

    delay(5);
  }

  stop();
}