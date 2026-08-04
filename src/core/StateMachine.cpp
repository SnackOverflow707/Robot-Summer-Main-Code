#include "core/StateMachine.h"
#include "comms/UART.h"

#include "tape_logic/TapeFollower.h"
#include "tape_logic/SideSensors.h"
#include "actuators/MecanumDrive.h"


// Expected mechanism files:
//
// mechanisms/RockGrabber.h
// mechanisms/TowerPieceGrabber.h
// mechanisms/TowerRam.h
// mechanisms/TowerBuilder.h
// mechanisms/TapeReturn.h
// mechanisms/IRAligner.h
// mechanisms/SolarPanelRipper.h
//
// Each mechanism should expose the functions used below.

#include "core/states/RockGrabber.h"
#include "core/states/SlowTapeFollowing.h"
#include "core/states/TowerRam.h"
#include "core/states/TowerBuilder.h"
#include "core/states/TapeReturn.h"
#include "core/states/IRAligner.h"
#include "core/states/IRAlignerManual.h"
#include "core/states/SolarPanelRipper.h"
#include "core/states/RockApproach.h"

extern MecanumDrive drive;

namespace StateMachine
{

// --------------------------------------------------
// Configuration
// --------------------------------------------------

static constexpr uint16_t MAG1_THRESHOLD = 20000;
static constexpr uint16_t MAG2_THRESHOLD = 3000;
static constexpr uint16_t METAL_THRESHOLD = 50;
static unsigned long courseStartTime = 0;


// How long to sit still and sample the metal detector before deciding.
// Longer = more confident, but costs time on every false alarm.
static constexpr unsigned long METAL_CHECK_WINDOW_MS = 300;

// Fraction of samples during the window that must read "metal detected"
// to count as a real hit. Filters out a single noisy blip.
static constexpr float METAL_CHECK_CONFIRM_RATIO = 0.6f;

static constexpr uint8_t NUM_ROCKS = 6;




static constexpr unsigned long SENSOR_DEBOUNCE_MS = 100;

static constexpr int SENSOR_SELECT_PIN = 11;

// --------------------------------------------------
// State variables
// --------------------------------------------------

static State currentState = State::STOPPED;
static unsigned long stateStartTime = 0;
static bool enabled = false;

static bool irTriggerArmed = true;
static bool metalTriggerArmed = true;
static bool returnTapeTriggerArmed = true;

static uint8_t sideTapeSightings = 0;
static bool sideTapeArmed = true;

// Which physical rock (0..NUM_ROCKS-1) we've stopped at. Increments on
// EVERY stop-and-check, whether or not that rock turns out to have
// metal - this is what lets ROCK_POSITIONS[] line up with the actual
// physical layout of all 6 rocks along the track.
static uint8_t rockIndex = 0;

// Sampling counters for the stopped metal-detector confirmation window.
static unsigned long metalCheckSampleCount = 0;
static unsigned long metalCheckHitCount = 0;

static unsigned long lastSideTapeTriggerTime = 0;
static unsigned long lastReturnTapeTriggerTime = 0;

// --------------------------------------------------
// State names and website IDs
// --------------------------------------------------

const char* getStateName(State state)
{
    switch (state)
    {
        case State::TAPE_FOLLOW_ROCK_CHECK:   return "Tape Follow + Rock Check";
        case State::ROCK_APPROACH:            return "Rock Approach";
        case State::ROCK_METAL_CHECK:         return "Rock Metal Check";
        case State::ROCK_GRAB:                return "Rock Grab";
        case State::TAPE_FOLLOW_TO_TOWER:     return "Tape Follow to Tower";
        case State::TOWER_RAM:                return "Tower Ram";
        case State::TOWER_BUILD:              return "Tower Build";
        case State::RETURN_TO_TAPE:           return "Return to Tape";
        case State::SLOW_TAPE_FOLLOWING:      return "Slow Tape Following";
        case State::IR_ALIGNING:         return "IR Tune Backward";
        case State::MANUAL_IR_ALIGNING:       return "Manual IR Aligning";
        case State::RIP_SOLAR_PANEL:          return "Rip Solar Panel";
        case State::ENDPOINT:                 return "Endpoint";
        case State::STOPPED:                  return "Stopped";
        default:                              return "Unknown";
    }
}

const char* getStateName()
{
    return getStateName(currentState);
}

const char* getStateId(State state)
{
    switch (state)
    {
        case State::TAPE_FOLLOW_ROCK_CHECK:   return "tape-rock";
        case State::ROCK_APPROACH:            return "rock-approach";
        case State::ROCK_METAL_CHECK:         return "rock-metal-check";
        case State::ROCK_GRAB:                return "rock-grab";
        case State::TAPE_FOLLOW_TO_TOWER:     return "tape-to-tower";
        case State::TOWER_RAM:                return "tower-ram";
        case State::TOWER_BUILD:              return "tower-build";
        case State::RETURN_TO_TAPE:           return "return-to-tape";
        case State::SLOW_TAPE_FOLLOWING:      return "slow-tape";
        case State::IR_ALIGNING:              return "ir-aligning";
        case State::MANUAL_IR_ALIGNING:       return "manual-ir-aligning";
        case State::RIP_SOLAR_PANEL:          return "rip-panel";
        case State::ENDPOINT:                 return "endpoint";
        case State::STOPPED:                  return "stopped";
        default:                              return "unknown";
    }
}

const char* getStateId()
{
    return getStateId(currentState);
}

// --------------------------------------------------
// Stop helpers
// --------------------------------------------------

static void stopAllMechanisms()
{
    RockApproach::stop();
    RockGrabber::stop();
    TapeReturn::stop();
    TowerBuilder::stop();
    SolarPanelRipper::stop();
    IRAligner::stop();
    IRAlignerManual::stop();
    TowerRam::stop();
    SlowTapeFollowing::stop();
}

static void stopCurrentOutputs()
{
    setTapeFollowing(false);
    drive.stop();
    stopAllMechanisms();
}

// --------------------------------------------------
// State transition
// --------------------------------------------------

static void changeState(State newState)
{
    stopCurrentOutputs();

    currentState = newState;
    stateStartTime = millis();

    Serial.print("State: ");
    Serial.println(getStateName());

    switch (currentState)
    {
        case State::TAPE_FOLLOW_ROCK_CHECK:
            resetTapePID();
            setTapeBaseSpeed(100);
            setTapeFollowing(true);
            break;

        case State::ROCK_APPROACH:
            RockApproach::begin();
            RockApproach::start(rockIndex);
            break;

        case State::ROCK_METAL_CHECK:
            drive.stop();
            delay(500);
            metalCheckSampleCount = 0;
            metalCheckHitCount = 0;
            break;

        case State::ROCK_GRAB:
            RockGrabber::begin();
            RockGrabber::start(rockIndex);
            break;


        case State::TAPE_FOLLOW_TO_TOWER:
            resetTapePID();
            setTapeBaseSpeed(120);
            setTapeFollowing(true);
            break;

        case State::TOWER_RAM:
            TowerRam::begin();
            TowerRam::start();
            break;

        case State::TOWER_BUILD:
            TowerBuilder::begin();
            TowerBuilder::start();
            break;

        case State::RETURN_TO_TAPE:
            TapeReturn::begin();
            TapeReturn::start();
            break;

        case State::SLOW_TAPE_FOLLOWING:
            resetTapePID();
            setTapeBaseSpeed(100);
            setTapeFollowing(true);
            SlowTapeFollowing::begin();
            SlowTapeFollowing::start();
            break;

        case State::IR_ALIGNING:

            IRAligner::begin();
            IRAligner::start();
            break;

        case State::MANUAL_IR_ALIGNING:
            IRAlignerManual::begin();
            IRAlignerManual::start();
            break;

        case State::RIP_SOLAR_PANEL:
            SolarPanelRipper::begin();
            SolarPanelRipper::start();
            break;

        case State::ENDPOINT:
        case State::STOPPED:
            drive.stop();
            break;
    }
}


static bool consumeReturnTapeTrigger(bool detected)
{
    if (!detected)
    {
        returnTapeTriggerArmed = true;
        return false;
    }

    if (!returnTapeTriggerArmed)
    {
        return false;
    }

    const unsigned long now = millis();

    if (now - lastReturnTapeTriggerTime < SENSOR_DEBOUNCE_MS)
    {
        return false;
    }

    returnTapeTriggerArmed = false;
    lastReturnTapeTriggerTime = now;

    return true;
}

// --------------------------------------------------
// Public control
// --------------------------------------------------

void begin()
{
    pinMode(SENSOR_SELECT_PIN, INPUT_PULLUP);

    enabled = false;
    sideTapeSightings = 0;
    rockIndex = 0;
    courseStartTime = millis();
    irTriggerArmed = true;
    metalTriggerArmed = true;
    sideTapeArmed = true;
    returnTapeTriggerArmed = true;

    changeState(State::STOPPED);
}

void setEnabled(bool value)
{
    if (value)
    {
        enabled = true;
        restart();
    }
    else
    {
        enabled = false;
        changeState(State::ENDPOINT);
    }
}

bool isEnabled()
{
    return enabled;
}

void restart()
{
    sideTapeSightings= 0;
    rockIndex = 0;

    irTriggerArmed = true;
    metalTriggerArmed = true;
    sideTapeArmed = true;
    returnTapeTriggerArmed = true;

    if (enabled)
    {
        changeState(State::ENDPOINT);
    }
    else
    {
        changeState(State::STOPPED);
    }
}

void stop()
{
    enabled = false;
    changeState(State::STOPPED);
}

// --------------------------------------------------
// Main update
// --------------------------------------------------

void update(const Inputs& inputs)
{
    if (!enabled)
    {
        return;
    }

    const bool irDetected =
        isSelectedDetected(inputs.mag1, inputs.mag2);

        const bool metalDetectedSensor0 =
        inputs.metalMagnitude0 > METAL_THRESHOLD;

    const bool metalDetectedSensor1 =
        inputs.metalMagnitude1 > METAL_THRESHOLD;

    // True if either detector sees metal.
    const bool metalDetected =
        metalDetectedSensor0 || metalDetectedSensor1;

    if (!irDetected)
    {
        irTriggerArmed = true;
    }

    if (!metalDetected)
    {
        metalTriggerArmed = true;
    }

    switch (currentState)
    {
        /*BELOW is commented out code which would work if the metal detectors
        had appropriate range.

        Because they don't, we are changing the logic so that the robot strafes right or left
        to each rock as it tape follows to check if they contain metal*/
        // case State::TAPE_FOLLOW_ROCK_CHECK:
        //     tapeFollowStep();

        //     if (metalDetected && metalTriggerArmed)
        //     {
        //         metalTriggerArmed = false;
        //         changeState(State::ROCK_METAL_CHECK);
        //         break;
        //     }
        //     break;

        //NEW LOGIC
    case State::TAPE_FOLLOW_ROCK_CHECK:
      {  tapeFollowStep();
        {
            auto pose = UART::getPoseData();
            if (pose.valid) {
                for (uint8_t i = rockIndex; i < NUM_ROCKS; i++) {
                    float dy = fabsf(pose.y - RockApproach::ROCK_POSITIONS[i].y);
                    if (dy < 20.0f) {
                        rockIndex = i;
                        changeState(State::ROCK_APPROACH);
                        break;
                    }
                }
            }
        }
        break;}

    case State::ROCK_APPROACH:{
        RockApproach::update();
        // keep tape following while waiting for Y position
        if (RockApproach::isFinished()) {
            changeState(State::ROCK_METAL_CHECK);
        } else if (RockApproach::hasFailed()) {
            changeState(State::ROCK_METAL_CHECK);
        }
        break;
    }

        case State::ROCK_METAL_CHECK:
        {
           /* ++metalCheckSampleCount;

            if (metalDetected)
            {
                ++metalCheckHitCount;
            }

            if (getStateElapsedMs() >= METAL_CHECK_WINDOW_MS)
            {
                const float hitRatio =
                    static_cast<float>(metalCheckHitCount) /
                    static_cast<float>(metalCheckSampleCount); */

                const RockApproach::RockPos& rp = RockApproach::ROCK_POSITIONS[rockIndex];

                bool onTape = false;
                const unsigned long returnStart = millis();

                if (rp.strafe)
                {
                    const unsigned long returnStart = millis();
                
                        UART::update();
                        updateTapeSensors();
                
                        const TapeFollowerStatus status =
                            getTapeFollowerStatus();
                
                        onTape =
                            !status.leftWhite ||
                            !status.rightWhite;
                
                        if (!onTape)
                        {
                            if (rp.coil == 0)
                            {
                                drive.strafeRight(150);
                            }
                            else
                            {
                                drive.strafeLeft(150);
                            }
                        }
                        else{
                            if (rockIndex < NUM_ROCKS - 1)
                            {
                                ++rockIndex;
                                changeState(State::TAPE_FOLLOW_ROCK_CHECK);
                            }
                            else {
                                changeState(State::TAPE_FOLLOW_TO_TOWER);
                            }   
                        }
                }
                else {
                    delay(500);
                    changeState(State::TAPE_FOLLOW_ROCK_CHECK);
                }
                break;
                
         }
        

        case State::ROCK_GRAB:{
            RockGrabber::update();

            if (RockGrabber::isFinished() || RockGrabber::hasFailed())
            {
                changeState(State::TAPE_FOLLOW_ROCK_CHECK);
            }
            break;
        }

            case State::TAPE_FOLLOW_TO_TOWER:
                {
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
                            sideTapeArmed = true;

                            changeState(State::TOWER_RAM);
                            break;
                        }
                    }

                    tapeFollowStep();
                    break;
                }


        case State::TOWER_RAM:

            TowerRam::update();

            if (TowerRam::isFinished())
            {
                changeState(State::TOWER_BUILD);
            }
            break;

        case State::TOWER_BUILD:
            TowerBuilder::update();

            if (TowerBuilder::isFinished())
            {
                changeState(State::RETURN_TO_TAPE);
            }
            break;

        case State::RETURN_TO_TAPE:
            TapeReturn::update();

            if (TapeReturn::isFinished())
            {
                changeState(State::SLOW_TAPE_FOLLOWING);
            }
            break;

        case State::SLOW_TAPE_FOLLOWING:
            SlowTapeFollowing::update();

            // SlowTapeFollowing now owns its own IR-detection and
            // panel-distance logic internally, and exposes which outcome
            // occurred via wasIRDetected()/needsManualFallback()/hasFailed()
            // -- route to the matching alignment path based on that instead
            // of re-checking inputs.mag1/mag2 here.
            if (SlowTapeFollowing::wasIRDetected())
            {
                changeState(State::IR_ALIGNING);
            }
            else if (SlowTapeFollowing::needsManualFallback())
            {
                changeState(State::MANUAL_IR_ALIGNING);
            }
            else if (SlowTapeFollowing::hasFailed())
            {
                changeState(State::STOPPED);
            }
        break;

        case State::IR_ALIGNING:

            IRAligner::update();

            if (IRAligner::isFinished())
            {
                changeState(State::RIP_SOLAR_PANEL);
            }
            else if (IRAligner::hasFailed())
            {
                changeState(State::STOPPED);
            }
            break;

        case State::MANUAL_IR_ALIGNING:

            IRAlignerManual::update();

            if (IRAlignerManual::isFinished())
            {
                // IRAlignerManual only gets the robot roughly onto the
                // panel (Y drive + hardcoded strafe) -- hand off to the
                // IR-sensor-driven aligner for accurate close-range
                // positioning instead of going straight to the ripper.


                //changeState(State::IR_ALIGNING); TEMPORARILY COMMENTED OUT FOR TESTING. 
                drive.stop(); 
            }
            else if (IRAlignerManual::hasFailed())
            {
                changeState(State::STOPPED);
            }

            break;

        case State::RIP_SOLAR_PANEL:
           SolarPanelRipper::update();

            if (SolarPanelRipper::isFinished())
            {
                changeState(State::ENDPOINT);
            }
            else if (SolarPanelRipper::hasFailed())
            {
                changeState(State::STOPPED);
            }

        break;
            break;

        case State::ENDPOINT:
        case State::STOPPED:
            drive.stop();
            break;
    }
}

// --------------------------------------------------
// Website/manual state selection
// --------------------------------------------------

bool requestState(State state)
{
    if (state == State::STOPPED)
    {
        stop();
        return true;
    }

    if (!enabled)
    {
        return false;
    }

    changeState(state);
    return true;
}

bool requestStateById(const String& stateId)
{
    if (stateId == "tape-rock")          return requestState(State::TAPE_FOLLOW_ROCK_CHECK);
    if (stateId == "rock-approach")      return requestState(State::ROCK_APPROACH);
    if (stateId == "rock-metal-check")   return requestState(State::ROCK_METAL_CHECK);
    if (stateId == "rock-grab")          return requestState(State::ROCK_GRAB);
    if (stateId == "tape-to-tower")      return requestState(State::TAPE_FOLLOW_TO_TOWER);
    if (stateId == "tower-ram")          return requestState(State::TOWER_RAM);
    if (stateId == "tower-build")        return requestState(State::TOWER_BUILD);
    if (stateId == "return-to-tape")     return requestState(State::RETURN_TO_TAPE);
    if (stateId == "slow-tape")          return requestState(State::SLOW_TAPE_FOLLOWING);
    if (stateId == "ir-aligning")          return requestState(State::IR_ALIGNING);
    if (stateId == "manual-ir-aligning") return requestState(State::MANUAL_IR_ALIGNING);
    if (stateId == "rip-panel")          return requestState(State::RIP_SOLAR_PANEL);
    if (stateId == "endpoint")           return requestState(State::ENDPOINT);
    if (stateId == "stopped")            return requestState(State::STOPPED);

    return false;
}

// --------------------------------------------------
// Telemetry
// --------------------------------------------------

State getState()
{
    return currentState;
}

unsigned long getStateElapsedMs()
{
    return millis() - stateStartTime;
}

uint8_t getSideTapeTriggerCount()
{
    return sideTapeSightings;
}

// --------------------------------------------------
// IR selection
// --------------------------------------------------

bool isMag1Selected()
{
    return digitalRead(SENSOR_SELECT_PIN) == LOW;
}

uint16_t getSelectedMagnitude(uint16_t mag1, uint16_t mag2)
{
    return isMag1Selected() ? mag1 : mag2;
}

bool isSelectedDetected(uint16_t mag1, uint16_t mag2)
{
    if (isMag1Selected())
    {
        return mag1 > MAG1_THRESHOLD;
    }

    return mag2 > MAG2_THRESHOLD;
}
unsigned long getCourseElapsedMs()
{
    return millis() - courseStartTime;
}
} // namespace StateMachine