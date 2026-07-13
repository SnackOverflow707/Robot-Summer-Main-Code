#include "actuators/MotorDriver.h"

static constexpr int PWM_FREQ = 1000;
static constexpr int PWM_RESOLUTION = 8;

MotorDriver::MotorDriver(int pinA, int pinB, int channelA, int channelB)
    : _pinA(pinA),
      _pinB(pinB),
      _channelA(channelA),
      _channelB(channelB)
{
}

void MotorDriver::begin() {
    ledcSetup(_channelA, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(_channelB, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(_pinA, _channelA);
    ledcAttachPin(_pinB, _channelB);

    stop();
}

void MotorDriver::setSpeed(int speed) {
    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        ledcWrite(_channelA, speed);
        ledcWrite(_channelB, 0);
    }
    else if (speed < 0) {
        ledcWrite(_channelA, 0);
        ledcWrite(_channelB, -speed);
    }
    else {
        stop();
    }
}

void MotorDriver::stop() {
    ledcWrite(_channelA, 0);
    ledcWrite(_channelB, 0);
}
