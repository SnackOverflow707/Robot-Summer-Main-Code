#pragma once
#include <Arduino.h>

class MotorDriver {
public:
  MotorDriver(int pinA, int pinB, int channelA, int channelB);

  void begin();
  void setSpeed(int speed);
  void stop();

private:
  int _pinA, _pinB;
  int _channelA, _channelB;
  int _lastSpeed;
};