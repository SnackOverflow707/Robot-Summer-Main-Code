#ifndef TAPEFOLLOWER_H
#define TAPEFOLLOWER_H

void tapeFollowStep();
bool isTapeFollowingEnabled();
void setTapeFollowing(bool enabled);
struct TapeFollowerStatus {
    float leftVoltage;
    float rightVoltage;
    bool leftWhite;
    bool rightWhite;
    float error;
    float pidOutput;
    float integral;
    float derivative;
    float kp;
    float ki;
    float kd;
};

TapeFollowerStatus getTapeFollowerStatus();

void setTapePID(float kp, float ki, float kd);
void resetTapePID();

#endif