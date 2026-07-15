#include "actuators/MotorDriver.h"

static constexpr int PWM_FREQ = 1000;
static constexpr int PWM_RESOLUTION = 8;
static constexpr int REVERSE_DEAD_TIME_US = 2000; // tune to your driver's datasheet

MotorDriver::MotorDriver(int pinA, int pinB, int channelA, int channelB)
    : _pinA(pinA),
      _pinB(pinB),
      _channelA(channelA),
      _channelB(channelB),
      _lastSpeed(0)
{
}

void MotorDriver::begin() {
    ledcSetup(_channelA, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(_channelB, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(_pinA, _channelA);
    ledcAttachPin(_pinB, _channelB);

    stop();
    _lastSpeed = 0;
}

void MotorDriver::setSpeed(int speed) {
    speed = constrain(speed, -255, 255);

    bool reversing = (speed > 0 && _lastSpeed < 0) || (speed < 0 && _lastSpeed > 0);

    if (reversing) {
        stop();
        delayMicroseconds(REVERSE_DEAD_TIME_US);
    }

    if (speed > 0) {
        ledcWrite(_channelA, speed);
        ledcWrite(_channelB, 0);
    } else if (speed < 0) {
        ledcWrite(_channelA, 0);
        ledcWrite(_channelB, -speed);
    } else {
        stop();
    }

    _lastSpeed = speed;
}

void MotorDriver::stop() {
    ledcWrite(_channelA, 0);
    ledcWrite(_channelB, 0);
}