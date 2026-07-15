#include "actuators/MecanumDrive.h"
#include "config/pins.h"

MecanumDrive::MecanumDrive()
  : frontLeft(FL_A, FL_B, 0, 1),
    frontRight(FR_A, FR_B, 2, 3),
    backLeft(BL_A, BL_B, 4, 5),
    backRight(BR_A, BR_B, 6, 7) {}

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