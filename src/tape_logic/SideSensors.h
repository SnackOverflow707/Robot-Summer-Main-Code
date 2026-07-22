#ifndef SIDESENSORS_H
#define SIDESENSORS_H

struct SideSensorStatus {
    float sensorVoltage;
    bool  onTape;
};

bool checkForSideTape();
SideSensorStatus getSideSensorStatus();

#endif