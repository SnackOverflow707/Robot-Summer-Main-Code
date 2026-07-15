#pragma once
#include "actuators/MotorDriver.h"

class MecanumDrive {
public:
  MecanumDrive();

  void begin();

  void forward(int speed);
  void backward(int speed);
  void strafeRight(int speed);
  void strafeLeft(int speed);
  void rotateClockwise(int speed);
  void rotateCounterClockwise(int speed);
  void stop();
  void frontLeftMotor(int speed);
  void frontRightMotor(int speed);
  void backLeftMotor(int speed);
  void backRightMotor(int speed);
  void rotateClockwiseBackAxis(int speed);
  void rotateCounterClockwiseBackAxis(int speed);

  void forwardWithRotateBackAxis(int fwdSpeed, int rotSpeed);
  void forwardWithRotate(int forwardSpeed, int rotateSpeed);

private:
  MotorDriver frontLeft;
  MotorDriver frontRight;
  MotorDriver backLeft;
  MotorDriver backRight;
};