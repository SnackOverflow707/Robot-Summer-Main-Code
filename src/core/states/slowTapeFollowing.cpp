#include "core/states/SlowTapeFollowing.h"

#include <Arduino.h>

#include "tape_logic/TapeFollower.h"
#include "tape_logic/SideSensors.h"
#include "actuators/MecanumDrive.h"

#include "comms/UART.h"

extern MecanumDrive drive;

namespace SlowTapeFollowing
{

static constexpr int TAPE_SPEED = 60;

static constexpr uint16_t MAG1_THRESHOLD = 20000;
static constexpr uint16_t MAG2_THRESHOLD = 3000;

static constexpr int SENSOR_SELECT_PIN = 11;

static bool running = false;
static bool sideTapesPassed = false; 

float currentX; 
float currentY; 

float sideTapeX; 
float sideTapeY;  

int sideTapeSightings; 

enum class SlowTapeFollowState
{
    IDLE,
    FOLLOWING, 
    DRIVE_TO_PANELS, 
    IR_DETECTED, 
    FINISHED,
    FAILED
};

static SlowTapeFollowState currentState = SlowTapeFollowState::IDLE;

void begin()
{
    pinMode(SENSOR_SELECT_PIN, INPUT_PULLUP);
    running = false;
}

void start()
{
    resetTapePID();
    setTapeBaseSpeed(TAPE_SPEED);
    setTapeFollowing(true);
    currentX = 0.0; 
    currentY = 0.0; 
    sideTapeSightings = 0; 

    running = true;
}

//checks if the incoming data from the Flow Sensor is valid. 
/*NOTE: we decided as a team to assume that the data is always fresh (aka updated very frequently) so we don't
check for freshness.*/
static bool isFlowSensorDataValid() {
    const UART::PoseData& flowData = UART::getPoseData();
    return (flowData.valid); 
}

static bool haveSideTapesPassed() {

    bool sideTapeArmed = false; //based on the idea that you need to LEAVE side tape first in order to detect both. 

    const SideSensorStatus sideStatus = getSideSensorStatus();
    // Sensor has returned to white, so allow another sighting.
    if (!sideStatus.onTape)
    {
        sideTapeArmed = true;
    }

    // Count only when we hit tape while armed.
    if (sideStatus.onTape && sideTapeArmed)
    {
        sideTapeArmed = false;
        sideTapeSightings++;

        if (sideTapeSightings >= 2)
        {
            sideTapeSightings = 0; 
            return true; 
        }
    }

}

static bool haveSolarPanelsPassed() {
    return ((sideTapeX - currentX <= SOLAR_PANEL_FROM_SIDE_TAPES_DX + SEARCH_THRESHOLD_X) 
    && (sideTapeY - currentY <= SOLAR_PANEL_FROM_SIDE_TAPES_DY + SEARCH_THRESHOLD_Y)); 
}


static void changeState(SlowTapeFollowState newState)
{
    drive.stop();

    currentState = newState;

    switch (currentState)
    {
        case SlowTapeFollowState::IDLE:
            break;

        case SlowTapeFollowState::FOLLOWING: 
            if (isIRDetected) {
                //switch to IR_DETECTED state 
                break; 
            }
            if (haveSolarPanelsPassed) {
                //switch to drive to panels state 
                break; 
            }
            break; 

        case SlowTapeFollowState::IR_DETECTED: 
            //successful; in the upper-level state machine (stateMachine.cpp), switch to IR_ALIGNING state. 
            break; 

        case SlowTapeFollowState::DRIVE_TO_PANELS: 
            //switch to finished state
            break;

        case SlowTapeFollowState::FINISHED: 
            break; 

        case SlowTapeFollowState::FAILED: 
            break; 

    }
}




bool isIRDetected(
    uint16_t mag1,
    uint16_t mag2
)
{
    const bool useMag1 =
        digitalRead(SENSOR_SELECT_PIN) == LOW;

    if (useMag1)
    {
        return mag1 > MAG1_THRESHOLD;
    }

    return mag2 > MAG2_THRESHOLD;
}

void update()
{
    if (!running)
    {
        return;
    }

    switch (currentState)
    {
        case SlowTapeFollowState::IDLE:
            break;

        case SlowTapeFollowState::FOLLOWING: 
            tapeFollowStep(); 
            //check if UART data is valid and record current position if so. 
            const UART::PoseData& flowData = UART::getPoseData();
            if (!isFlowSensorDataValid()) {
                //return false; switch to failed state or like... try again...? maybe have a certain number of "allowed" invalid samples before calling failure?
            }
            else {
                currentX = flowData.x; 
                currentY = flowData.y; 

                if (haveSideTapesPassed() && !sideTapesPassed) {
                    sideTapeX = currentX; 
                    sideTapeY = currentY;
                    sideTapesPassed = true; 
                }

                else if (isIRDetected) {
                    changeState(SlowTapeFollowState::IR_DETECTED); 
                    break; 
                }

                else if (havePanelsPassed) {
                    changeState(SlowTapeFollowState::DRIVE_TO_PANELS); 
                    break; 
                }

            }
        
        case SlowTapeFollowState::IR_DETECTED:
            changeState(SlowTapeFollowState::FINISHED); 
            break; 

        /*case SlowTapeFollowState::DRIVE_TO_PANELS:
            changeState()*/
    }



}


void stop()
{
    setTapeFollowing(false);
    running = false;
}





} // namespace SlowTapeFollowing