#include "core/StateMachine.h"
#include "comms/UART.h"

#include "tape_logic/TapeFollower.h"
#include "tape_logic/SideSensors.h"
#include "actuators/MecanumDrive.h"
#include "core/states/RockMetalCheck.h"


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

static constexpr uint16_t METAL_THRESHOLD = 150;
static unsigned long courseStartTime = 0;


// How long to sit still and sample the metal detector before deciding.
// Longer = more confident, but costs time on every false alarm.
static constexpr unsigned long METAL_CHECK_WINDOW_MS = 300;

// Fraction of samples during the window that must read "metal detected"
// to count as a real hit. Filters out a single noisy blip.
static constexpr float METAL_CHECK_CONFIRM_RATIO = 0.6f;

static constexpr uint8_t NUM_ROCKS_TO_CHECK = 1;
static constexpr uint8_t LAST_ROCK_INDEX = NUM_ROCKS_TO_CHECK - 1;

static constexpr int UP_RAMP_TAPE_SPEED = 160;
static constexpr unsigned long UP_RAMP_TIME_MS = 25000;

// Open-loop backup before the fallback solar panel grab sequence.
// Matches IRAligner's SEARCH_FORWARD speed/time (IRAligner.cpp), since
// that's the search that just failed right before this runs.
static constexpr int FALLBACK_BACKUP_SPEED = 60;
static constexpr unsigned long FALLBACK_BACKUP_TIME_MS = 3000;



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
static bool isMetal = false;


//Metal detector variables 
static constexpr uint8_t METAL_SAMPLE_COUNT = 5;
static constexpr unsigned long METAL_SETTLE_TIME_MS = 00;

// Tune this using your measured detector values.
static constexpr float METAL_CHANGE_THRESHOLD_HZ = 100.0f;

static float metalBaselineSum = 0.0f;
static float metalBaselineAverage = 0.0f;
static uint8_t metalBaselineCount = 0;
static bool metalBaselineReady = false;

static float metalCheckSum = 0.0f;
static uint8_t metalCheckCount = 0;

// Tune this after looking at the actual frequency changes.
static float getRockMetalReading(
    const Inputs& inputs,
    uint8_t rock
);

static float metalCheckBaselineHz = 0.0f;
static float metalCheckMaxChangeHz = 0.0f;
static bool metalCheckInitialized = false;

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
        case State::TAPE_FOLLOW_UP_RAMP:   return "Tape follow up ramp";
        case State::TAPE_FOLLOW_TO_TOWER:     return "Tape Follow to Tower";
        case State::TOWER_RAM:                return "Tower Ram";
        case State::TOWER_BUILD:              return "Tower Build";
        case State::RETURN_TO_TAPE:           return "Return to Tape";
        case State::SLOW_TAPE_FOLLOWING:      return "Slow Tape Following";
        case State::IR_ALIGNING:         return "IR Tune Backward";
        case State::MANUAL_IR_ALIGNING:       return "Manual IR Aligning";
        case State::RIP_SOLAR_PANEL:          return "Rip Solar Panel";
        case State::FALLBACK_BACKUP:          return "Fallback Backup";
        case State::RIP_SOLAR_PANEL_FALLBACK: return "Rip Solar Panel (Fallback)";
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
        case State::TAPE_FOLLOW_UP_RAMP:      return "tape-up-ramp";
        case State::TAPE_FOLLOW_TO_TOWER:     return "tape-to-tower";
        case State::TOWER_RAM:                return "tower-ram";
        case State::TOWER_BUILD:              return "tower-build";
        case State::RETURN_TO_TAPE:           return "return-to-tape";
        case State::SLOW_TAPE_FOLLOWING:      return "slow-tape";
        case State::IR_ALIGNING:              return "ir-aligning";
        case State::MANUAL_IR_ALIGNING:       return "manual-ir-aligning";
        case State::RIP_SOLAR_PANEL:          return "rip-panel";
        case State::FALLBACK_BACKUP:          return "fallback-backup";
        case State::RIP_SOLAR_PANEL_FALLBACK: return "rip-panel-fallback";
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
    const State previousState = currentState;

    stopCurrentOutputs();

    currentState = newState;
    stateStartTime = millis();

    Serial.print("State: ");
    Serial.println(getStateName());

    switch (currentState)
    {
        case State::TAPE_FOLLOW_ROCK_CHECK:
            resetTapePID();
            setTapeBaseSpeed(160);
            setTapeFollowing(true);
            RockMetalCheck::resetBaselines();
            break;

        case State::ROCK_APPROACH:
            metalBaselineSum = 0.0f;
            metalBaselineAverage = 0.0f;
            metalBaselineCount = 0;
            metalBaselineReady = false;
        
            RockApproach::begin();
            // Do not start strafing until baseline is ready.
            break;

        case State::ROCK_METAL_CHECK:
            RockMetalCheck::begin();
            RockMetalCheck::start(rockIndex);
            break;
            
        case State::ROCK_GRAB:
            RockGrabber::begin();
            RockGrabber::start(rockIndex);
            break;

        case State::TAPE_FOLLOW_UP_RAMP:
            resetTapePID();
            setTapeBaseSpeed(UP_RAMP_TAPE_SPEED);
            setTapeFollowing(true);
            break;


        case State::TAPE_FOLLOW_TO_TOWER:
            resetTapePID();
            setTapeBaseSpeed(120);
            setTapeFollowing(true);
            sideTapeSightings = 0;
            sideTapeArmed = true;
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
            sideTapeSightings = 0;
            sideTapeArmed = true;
            SlowTapeFollowing::begin();
            SlowTapeFollowing::start();
            break;

        case State::IR_ALIGNING:
        {
            // Coming from MANUAL_IR_ALIGNING means IRAlignerManual already
            // drove Y and hardcode-strafed onto the panel -- skip IRAligner's
            // own initial strafe and go straight to the backward/forward
            // peak search. Coming straight from SLOW_TAPE_FOLLOWING (IR
            // detected mid-tape-follow) means no strafe has happened yet,
            // so keep the normal strafe-then-search sequence.
            const bool skipInitialStrafe =
                (previousState == State::MANUAL_IR_ALIGNING);
            IRAligner::begin();
            IRAligner::start(skipInitialStrafe);
            break;
        }

        case State::MANUAL_IR_ALIGNING:
            IRAlignerManual::begin();
            IRAlignerManual::start();
            break;

        case State::RIP_SOLAR_PANEL:
            SolarPanelRipper::begin();
            SolarPanelRipper::start();
            break;

        case State::FALLBACK_BACKUP:
            // Driven open-loop in update(); nothing to set up here.
            break;

        case State::RIP_SOLAR_PANEL_FALLBACK:
            SolarPanelRipper::begin();
            SolarPanelRipper::startFallback();
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
    RockMetalCheck::resetBaselines();
    isMetal = false;

    changeState(State::ENDPOINT);
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
    courseStartTime = millis();

    irTriggerArmed = true;
    metalTriggerArmed = true;
    sideTapeArmed = true;
    returnTapeTriggerArmed = true;
    RockMetalCheck::resetBaselines();
    isMetal = false;

    if (enabled)
    {
        UART::resetFlowPose();
        changeState(State::TAPE_FOLLOW_UP_RAMP);
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
    if (!RockMetalCheck::areBaselinesReady())
    {
    drive.stop();
    setTapeFollowing(false);

    RockMetalCheck::updateBaselines(inputs);

    if (RockMetalCheck::areBaselinesReady())
    {
        resetTapePID();
        setTapeBaseSpeed(100);
        setTapeFollowing(true);
    }

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
                for (uint8_t i = rockIndex; i < NUM_ROCKS_TO_CHECK; i++) {
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

        case State::ROCK_APPROACH:
        {
            if (!metalBaselineReady)
            {
                const float reading =
                    getRockMetalReading(inputs, rockIndex);
        
                metalBaselineSum += reading;
                ++metalBaselineCount;
        
                Serial.printf(
                    "Baseline sample %u/5: %.2f Hz\n",
                    metalBaselineCount,
                    reading
                );
        
                if (metalBaselineCount >= METAL_SAMPLE_COUNT)
                {
                    metalBaselineAverage =
                        metalBaselineSum /
                        static_cast<float>(METAL_SAMPLE_COUNT);
        
                    metalBaselineReady = true;
        
                    Serial.printf(
                        "Rock %u baseline: %.2f Hz\n",
                        rockIndex,
                        metalBaselineAverage
                    );
        
                    // Now begin moving toward the rock.
                    RockApproach::start(rockIndex);
                }
        
                break;
            }
        
            RockApproach::update();
        
            if (
                RockApproach::isFinished() ||
                RockApproach::hasFailed()
            )
            {
                changeState(State::ROCK_METAL_CHECK);
            }
        
            break;
        }

        case State::ROCK_METAL_CHECK:
        {
            RockMetalCheck::update(inputs);
        
            if (RockMetalCheck::hasFailed())
            {
                changeState(State::STOPPED);
                break;
            }
        
            if (!RockMetalCheck::isFinished())
            {
                break;
            }
        
            const bool isFinalCheckedRock =rockIndex == LAST_ROCK_INDEX;
            const bool shouldGrab = RockMetalCheck::metalFound() || (isFinalCheckedRock && !isMetal);
        if (shouldGrab)
        {
            isMetal = true;
            changeState(State::ROCK_GRAB);
            break;
        }
        
            const RockApproach::RockPos& rock =
                RockApproach::ROCK_POSITIONS[rockIndex];
        
            updateTapeSensors();
        
            const TapeFollowerStatus status =
                getTapeFollowerStatus();
        
            const bool onTape =
                !status.leftWhite ||
                !status.rightWhite;
        
            if (!onTape)
            {
                if (rock.coil == 1 || rockIndex == 3 || rockIndex ==2)
                {
                    drive.strafeLeft(150);
                }
                else
                {
                    drive.strafeRight(150);
                }
        
                break;
            }
        
            drive.stop();
        
            if (rockIndex < LAST_ROCK_INDEX)
{
    ++rockIndex;
    changeState(State::TAPE_FOLLOW_ROCK_CHECK);
}
else
{
    // Normally unreachable because the final checked
    // rock is always grabbed, but this is a safe fallback.
    changeState(State::TAPE_FOLLOW_UP_RAMP);
}
        
            break;
        }

        case State::ROCK_GRAB:{
            RockGrabber::update();

            if (RockGrabber::isFinished() || RockGrabber::hasFailed())
            {
                changeState(State::TAPE_FOLLOW_UP_RAMP);
            }
            break;
        }
        case State::TAPE_FOLLOW_UP_RAMP:
{
    tapeFollowStep();

    if (
        millis() - stateStartTime >=
        UP_RAMP_TIME_MS
    )
    {
        changeState(State::TAPE_FOLLOW_TO_TOWER);
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
                // Never found the beacon precisely -- the robot is not
                // in the normal aligned position. Back off the panel,
                // then run the fallback grab sequence instead of just
                // stopping.
                changeState(State::FALLBACK_BACKUP);
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


                changeState(State::IR_ALIGNING);
                drive.stop(); 
            }
            else if (IRAlignerManual::hasFailed())
            {
                changeState(State::STOPPED);
            }

            break;

        case State::FALLBACK_BACKUP:
        {
            // Open-loop: no pose check, just run for a fixed time.
            if (millis() - stateStartTime >= FALLBACK_BACKUP_TIME_MS)
            {
                drive.stop();
                changeState(State::RIP_SOLAR_PANEL_FALLBACK);
                break;
            }

            drive.backward(FALLBACK_BACKUP_SPEED);
            break;
        }

        case State::RIP_SOLAR_PANEL:
        case State::RIP_SOLAR_PANEL_FALLBACK:
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
    if (stateId == "ir-aligning")        return requestState(State::IR_ALIGNING);
    if (stateId == "tape-up-ramp")       return requestState(State::TAPE_FOLLOW_UP_RAMP);
    if (stateId == "manual-ir-aligning") return requestState(State::MANUAL_IR_ALIGNING);
    if (stateId == "rip-panel")          return requestState(State::RIP_SOLAR_PANEL);
    if (stateId == "fallback-backup")    return requestState(State::FALLBACK_BACKUP);
    if (stateId == "rip-panel-fallback") return requestState(State::RIP_SOLAR_PANEL_FALLBACK);
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
    // Delegates to SlowTapeFollowing's thresholds -- that's the only
    // place MAG1_THRESHOLD/MAG2_THRESHOLD are defined now, since that's
    // what actually drives IR-detection behavior; this just mirrors it
    // for the website's telemetry display.
    return SlowTapeFollowing::isIRDetected(mag1, mag2);
}
unsigned long getCourseElapsedMs()
{
    return millis() - courseStartTime;
}
static float getRockMetalReading(
    const Inputs& inputs,
    uint8_t rock
)
{
    const RockApproach::RockPos& rp =
        RockApproach::ROCK_POSITIONS[rock];

    return (rp.coil == 0)
        ? inputs.metalMagnitude1   // left detector
        : inputs.metalMagnitude0;  // right detector
}
bool getIsMetal()
{
    return isMetal;
}
} // namespace StateMachine