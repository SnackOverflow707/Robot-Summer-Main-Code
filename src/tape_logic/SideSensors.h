#ifndef SIDESENSORS_H
#define SIDESENSORS_H

struct SideSensorStatus {
    float sensorVoltage;
    bool  onTape;
};

void checkForSideTape();
SideSensorStatus getSideSensorStatus();

#endif