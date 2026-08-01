#include "core/states/IRAligner.h"

#include <Arduino.h>

#include "actuators/MecanumDrive.h"
#include "comms/UART.h"

extern MecanumDrive drive;

#define SOLAR_PANEL_FROM_TOWER_DX 
#define SOLAR_PANEL_FROM_TOWER_DY 

namespace IRAlignerManual 
{

    /* What needs to happen: 
    - robot records position when tower building 
    - during slow tape following, robot should constantly be recording position relative to tower building state position
    - if the avg distance threshold + travel buffer is exceeded, fall back on manual solar panel alignment based on 
    measured coordinates from the tower 
    - make the driveToPanel function and call it in StateMachine.cpp when the robot is in the manual IR aligning state
    */


}; // namespace IRAlignerManual