#include "core/states/IRAligner.h"

#include <Arduino.h>

#include "actuators/MecanumDrive.h"
#include "comms/UART.h"

extern MecanumDrive drive;

namespace IRAligner
{

// --------------------------------------------------
// Configuration
// --------------------------------------------------

// Initial right strafe
static constexpr int STRAFE_SPEED = 90;
static constexpr unsigned long STRAFE_TIME_MS = 1600;


// --------------------------------------------------
// Search motion
// --------------------------------------------------

// First move backward far enough to get to one side
// of the directional IR peak.
static constexpr int BACKWARD_SPEED = 60;
static constexpr unsigned long BACKWARD_TIME_MS = 2000;

// Then scan forward slowly across the IR peak.
static constexpr int FORWARD_SCAN_SPEED = 60;

// Maximum time allowed for the forward peak search.
static constexpr unsigned long FORWARD_SEARCH_TIME_MS = 2000;


// --------------------------------------------------
// Peak detection
// --------------------------------------------------

// Signal must reach at least this value before we consider
// the observed maximum to be a real IR peak.
//
// These are NOT "stop here" thresholds.
// They only reject noise / weak background signals.
static constexpr uint16_t MAG1_MIN_VALID_PEAK = 10000;
static constexpr uint16_t MAG2_MIN_VALID_PEAK = 3000;

// Once the signal falls below this fraction of the best
// signal seen, we assume we have passed the peak.
//
// Example:
// peak = 3000
// 0.75 * 3000 = 2250
//
// Once the signal is below 2250 for several samples,
// we consider the peak passed.
static constexpr float PEAK_DROP_RATIO = 0.75f;

// Require several consecutive falling readings so one
// noisy sample does not cause a reversal.
static constexpr uint8_t REQUIRED_FALLING_SAMPLES = 4;


// --------------------------------------------------
// Return to peak
// --------------------------------------------------

// Use the same speed in reverse so time is approximately
// proportional to distance.
static constexpr int RETURN_SPEED = FORWARD_SCAN_SPEED;

// Don't allow an unexpectedly huge return movement.
static constexpr unsigned long MAX_RETURN_TIME_MS = 1200;


// --------------------------------------------------
// UART
// --------------------------------------------------

static constexpr unsigned long UART_TIMEOUT_MS = 250;


// --------------------------------------------------
// Filtering
// --------------------------------------------------

// Exponential smoothing.
//
// Higher -> reacts faster, more noise
// Lower  -> smoother, more lag
static constexpr float FILTER_ALPHA = 0.30f;


// --------------------------------------------------
// IR channel selection
// --------------------------------------------------

// true  = use mag2
// false = use mag1
static constexpr bool USE_MAG2 = true;


// --------------------------------------------------
// Internal states
// --------------------------------------------------

enum class IRAlignState
{
    IDLE,
    SEARCH_FOR_PEAK, 
    RETURN_TO_PEAK,
    ALIGN_ROBOT, 
    FINISHED,
    NOT_FOUND
};


static IRAlignState currentState = IRAlignState::IDLE;

static unsigned long stateStartTime = 0;


// --------------------------------------------------
// Filter state
// --------------------------------------------------

static float filteredMagnitude = 0.0f;
static bool filterInitialized = false;


// --------------------------------------------------
// Peak tracking
// --------------------------------------------------

static uint16_t maximumMagnitude = 0;

// Time at which the strongest signal was observed
// during the forward scan.
static unsigned long peakTime = 0;

// How long we continued forward after the peak.
// We reverse for approximately this long.
static unsigned long returnTimeMs = 0;

// Consecutive samples sufficiently below the peak.
static uint8_t fallingSampleCount = 0;

// True once we've seen a signal strong enough to
// consider the peak real.
static bool validPeakSeen = false;


// --------------------------------------------------
// IR helpers
// --------------------------------------------------

static uint16_t getMinimumValidPeak()
{
    return USE_MAG2
        ? MAG2_MIN_VALID_PEAK
        : MAG1_MIN_VALID_PEAK;
}


static bool uartDataIsFresh()
{
    const UART::Data& data = UART::getData();

    if (!data.valid)
    {
        return false;
    }

    return millis() - data.lastUpdateMs <= UART_TIMEOUT_MS;
}

//checks if the incoming data from the Flow Sensor is valid. 
/*NOTE: we decided as a team to assume that the data is always fresh (aka updated very frequently) so we don't
check for freshness.*/
static bool isFlowSensorDataValid() {
    const UART::PoseData& flowData = UART::getPoseData();
    return (flowData.valid); 
}


static uint16_t getRawIRMagnitude()
{
    const UART::Data& data = UART::getData();

    if (!uartDataIsFresh())
    {
        return 0;
    }

    return USE_MAG2
        ? data.mag2
        : data.mag1;
}


// --------------------------------------------------
// Filter
// --------------------------------------------------

static void resetDetectionFilter()
{
    filteredMagnitude = 0.0f;
    filterInitialized = false;
}


static void updateDetectionFilter()
{
    if (!uartDataIsFresh())
    {
        filterInitialized = false;
        filteredMagnitude = 0.0f;
        return;
    }

    const uint16_t rawMagnitude =
        getRawIRMagnitude();

    if (!filterInitialized)
    {
        filteredMagnitude =
            static_cast<float>(rawMagnitude);

        filterInitialized = true;
    }
    else
    {
        filteredMagnitude =
            FILTER_ALPHA *
                static_cast<float>(rawMagnitude)
            +
            (1.0f - FILTER_ALPHA) *
                filteredMagnitude;
    }
}


static uint16_t getFilteredMagnitude()
{
    if (!filterInitialized)
    {
        return 0;
    }

    return static_cast<uint16_t>(
        filteredMagnitude
    );
}



/* GOAL FOR SENSOR INTEGRATION: 
- Instead of using time passed since peak detection to backtrack, record distance instead
- then drive the robot back to the location where the peak was detected 

Functions/states needed: 
- STATE: searchForPeak
- STATE: peakFound 
    - FUNCTION: tracks the distance passed since peak detection and the actual physical location of the beacon, 
    based on the robot's speed 
- STATE: driveBackToPeak 
- STATE: alignRobot (get to a consistent position: eg. rotate so parallel to panels so the arm knows where to go) 

we also need the general switch state function that tells the robot which state to switch to after certain events. 

*/

/*in general begin state: 
- initialize the filter
- reset max peak magnitude detected to 0 */

/*In general loop state:
- update the filter */

/*in searchForPeak:
- feed filter new data  
- capture current distance
- analyze the filter's output against running maximums 
    - if it's a new maximum, record it & the robot's location atp
- analyze the filter's output against the falling threshold 
    - if it's below threshold, it means the peak as passed --> trigger the peakFound state 
*/

//static float peakForwardDistance = 0.0f; //meters
//static float overshootDistance  = 0.0f; //meters

//static constexpr float MAX_RETURN_DISTANCE = 0.30f;  // safety cap
static constexpr float RETURN_STOP_EPSILON = 0.01f;  // 1cm tolerance

static float xCurrent = 0.0f; 
static float yCurrent = 0.0f; 
static float xOfMax = 0.0f; 
static float yOfMax = 0.0f; 
static float dxToPeak = 0.0f; 
static float dyToPeak = 0.0f; 

//searchForPeak functions 

static void resetPeakTracking() {
    maximumMagnitude = 0;
    fallingSampleCount = 0;
    validPeakSeen = false;
    dxToPeak = 0.0f;
    
    //peakForwardDistance = 0.0f;
    //overshootDistance = 0.0f;
}


//ONLY RETURNS TRUE WHEN A PEAK IS DETECTED AND THE ROBOT NEEDS TO DRIVE BACK. 
static bool hasPeakPassed() {

    //calculate falling threshold based on running max mag 
    const float fallingThreshold = static_cast<float>(maximumMagnitude) * PEAK_DROP_RATIO;

    //check filter initialization 
    if (!filterInitialized) {
        fallingSampleCount = 0;
        return false;
    }

    //get new data from UART & integrate data to filter 
    const uint16_t magnitude = getFilteredMagnitude();
    const UART::Data& data = UART::getData();

    //check if UART data is valid and record current position if so. 
    const UART::PoseData& flowData = UART::getPoseData();
    if (!isFlowSensorDataValid()) {
        return false; 
    }
    xCurrent = flowData.x; 
    yCurrent = flowData.y; 

    //check if current magnitude is a maximum. 
    //if maximum, store it and store the x/y coords
    if (magnitude > maximumMagnitude) {

        maximumMagnitude = magnitude; 

        if (!(maximumMagnitude >= getMinimumValidPeak())) {
            validPeakSeen = false; 
            return false; 
        }

        validPeakSeen = true;
        xOfMax = xCurrent; 
        yOfMax = yCurrent; 
        fallingSampleCount = 0;
        return false; //updated the max only, not necessarily a peak yet. 
    }

    //else, check if signal has passed below threshold 
    else {
        
        if (magnitude < fallingThreshold) {
            
            //first check if a peak has been seen
            if (!validPeakSeen) {
                fallingSampleCount = 0;
                return false;
            }

            //check if required number of falling samples has been met
            if (fallingSampleCount < REQUIRED_FALLING_SAMPLES) {
                fallingSampleCount++;
            }
        }

        else {
            fallingSampleCount = 0;
        }

        //if samples has been met, save relevant data 
        if (fallingSampleCount >= REQUIRED_FALLING_SAMPLES) {
            dxToPeak = xCurrent - xOfMax; 
            dyToPeak = yCurrent - yOfMax; 
            return true; 
        }
        //else continue search 
        return false; 
    }

    return false; 

}

void alignRobot() {
    //will need to calibrate stuff based off where the robot stops after RETURN_TO_PEAK;
    //can't assume it's perfectly in front of the beacon. Just measure or something. 

    //will also need to calibrate the degrees for rotation; figure out what orientation 
    //the panels are in the sensors' world. 

}


//state switching 
static void changeState(IRAlignState newState) {

    drive.stop();
    currentState = newState; 
    stateStartTime = millis();
    
    switch (currentState)
    {
        case IRAlignState::IDLE: {
            break;
        }
        
        case IRAlignState::SEARCH_FOR_PEAK: {
            // Deliberately not resetting the filter/peak tracking here:
            // tracking may already be underway from Slow Tape Following
            // (see begin()), and resetting would throw that away.
            drive.forward(FORWARD_SCAN_SPEED);
            break;
        }

        case IRAlignState::RETURN_TO_PEAK: {
            //gotta drive until we satisfy dxToMax and dyToMax, then break
            drive.driveTo(dxToPeak, dyToPeak, RETURN_SPEED); 
            break; 
        }

        case IRAlignState::ALIGN_ROBOT: {

            //add function 
            break; 
        }


        case IRAlignState::FINISHED:
        case IRAlignState::NOT_FOUND: {
            drive.stop();
            break;
        }

    }

}



//one-time startup upon booting 
void begin() {
    currentState = IRAlignState::IDLE;
    stateStartTime = 0;
    resetDetectionFilter();
    resetPeakTracking();
    drive.stop();
}

//trigger an IRAligning attempt. NOTE: this is called in the upper-level state machine ONCE A VALID PEAK HAS BEEN DETECTED.
void start() {
    // If we were already tracking (and passed) the peak while idle
    // during Slow Tape Following, go straight back to it instead of
    // resetting and blindly scanning forward again.
    const bool peakAlreadyPassed =
        validPeakSeen && fallingSampleCount >= REQUIRED_FALLING_SAMPLES;

    changeState(peakAlreadyPassed
        ? IRAlignState::RETURN_TO_PEAK
        : IRAlignState::SEARCH_FOR_PEAK);
}


void update()
{
    // Keep the filter fed every tick, regardless of state 
    updateDetectionFilter();

    switch (currentState)
    {
        case IRAlignState::IDLE: {
            // Track the signal even before formally entering IR
            // aligning, so a peak passed during Slow Tape Following
            // isn't missed. Ignore the return value here -- the
            // top-level state machine still decides when to act on it.
            hasPeakPassed();
            break;
        }

        case IRAlignState::SEARCH_FOR_PEAK: {
            if (hasPeakPassed()) {
                changeState(IRAlignState::RETURN_TO_PEAK);
                break;
            }
            if (millis() - stateStartTime >= FORWARD_SEARCH_TIME_MS) {
                changeState(IRAlignState::NOT_FOUND);
            }
            break;
        }

        case IRAlignState::RETURN_TO_PEAK:
        {
            if (isFlowSensorDataValid()) {
                const UART::PoseData& flowData = UART::getPoseData();
                const float dx = flowData.x - xOfMax;
                const float dy = flowData.y - yOfMax;
                const float distanceToPeak = sqrtf(dx * dx + dy * dy);

                if (distanceToPeak <= RETURN_STOP_EPSILON) {
                    changeState(IRAlignState::ALIGN_ROBOT);
                    break;
                }
            }

            // Safety fallback
            if (millis() - stateStartTime >= MAX_RETURN_TIME_MS)
            {
                changeState(IRAlignState::NOT_FOUND);
            }

            break;
        }

        case IRAlignState::ALIGN_ROBOT:
        {
            alignRobot(); 
            changeState(IRAlignState::FINISHED);
            break;
        }

        case IRAlignState::FINISHED:
        case IRAlignState::NOT_FOUND:
        {
            drive.stop();
            break;
        }
    }
}

void stop() {
    changeState(IRAlignState::IDLE);
}



//status functions 
bool isFinished()
{
    return
        currentState ==
        IRAlignState::FINISHED;
}


bool hasFailed()
{
    return
        currentState ==
        IRAlignState::NOT_FOUND;
}


bool isDone()
{
    return
        isFinished() ||
        hasFailed();
}


//state names 
const char* getStateName() {
        switch (currentState)
    {
        case IRAlignState::IDLE:
            return "Idle";

        case IRAlignState::SEARCH_FOR_PEAK: 
            return "Searching for peak signal"; 
        
        case IRAlignState::ALIGN_ROBOT: 
            return "Aligning the robot to the solar panels"; 

        case IRAlignState::RETURN_TO_PEAK:
            return "Return to Peak";

        case IRAlignState::FINISHED:
            return "IR Aligned";

        case IRAlignState::NOT_FOUND:
            return "IR Peak Not Found";
    }

    return "Unknown";
}


// for the UI 
uint16_t getCurrentMagnitude()
{
    return getFilteredMagnitude();
}


uint16_t getMaximumMagnitude()
{
    return maximumMagnitude;
}




/* 
-----------------------------------------------------------------------------------------------------
OLD CODE 

// --------------------------------------------------
// Peak tracking
// --------------------------------------------------


static void resetPeakTracking()
{
    maximumMagnitude = 0;

    peakTime = 0;
    returnTimeMs = 0;

    fallingSampleCount = 0;

    validPeakSeen = false;
}


static bool updatePeakTracking()
{
    if (!filterInitialized)
    {
        fallingSampleCount = 0;
        return false;
    }

    const uint16_t magnitude =
        getFilteredMagnitude();


    // --------------------------------------------------
    // New maximum found
    // --------------------------------------------------

    if (magnitude > maximumMagnitude)
    {
        maximumMagnitude = magnitude;

        peakTime = millis();

        fallingSampleCount = 0;

        if (
            maximumMagnitude >=
            getMinimumValidPeak()
        )
        {
            validPeakSeen = true;
        }

        return false;
    }


    // --------------------------------------------------
    // Don't look for a falling edge until we've seen
    // a legitimate peak.
    // --------------------------------------------------

    if (!validPeakSeen)
    {
        return false;
    }


    // --------------------------------------------------
    // Determine how far below the peak we currently are
    // --------------------------------------------------

    const float fallingThreshold =
        static_cast<float>(maximumMagnitude) *
        PEAK_DROP_RATIO;


    if (
        static_cast<float>(magnitude) <
        fallingThreshold
    )
    {
        if (
            fallingSampleCount <
            REQUIRED_FALLING_SAMPLES
        )
        {
            ++fallingSampleCount;
        }
    }
    else
    {
        fallingSampleCount = 0;
    }


    // --------------------------------------------------
    // Peak has been passed
    // --------------------------------------------------

    if (
        fallingSampleCount >=
        REQUIRED_FALLING_SAMPLES
    )
    {
        returnTimeMs =
            millis() - peakTime;

        // Limit the reverse correction.
        if (
            returnTimeMs >
            MAX_RETURN_TIME_MS
        )
        {
            returnTimeMs =
                MAX_RETURN_TIME_MS;
        }

        return true;
    }


    return false;
}


// --------------------------------------------------
// State transition
// --------------------------------------------------

static void changeState(AlignState newState)
{
    drive.stop();

    currentState = newState;
    stateStartTime = millis();


    switch (currentState)
    {
        // --------------------------------------------------

        case AlignState::IDLE:
        {
            break;
        }


        // --------------------------------------------------

        case AlignState::STRAFE_RIGHT:
        {
            drive.strafeRight(
                STRAFE_SPEED
            );

            break;
        }


        // --------------------------------------------------

        case AlignState::SEARCH_BACKWARD:
        {
            drive.backward(
                BACKWARD_SPEED
            );

            break;
        }


        // --------------------------------------------------

        case AlignState::SEARCH_FORWARD:
        {
            // Important:
            // reset peak tracking immediately before
            // the actual precision sweep.
            resetPeakTracking();

            drive.forward(
                FORWARD_SCAN_SPEED
            );

            break;
        }


        // --------------------------------------------------

        case AlignState::RETURN_TO_PEAK:
        {
            drive.backward(
                RETURN_SPEED
            );

            break;
        }


        // --------------------------------------------------

        case AlignState::FINISHED:
        case AlignState::NOT_FOUND:
        {
            drive.stop();
            break;
        }
    }
}


// --------------------------------------------------
// Public functions
// --------------------------------------------------

void begin()
{
    currentState =
        AlignState::IDLE;

    stateStartTime = 0;

    resetDetectionFilter();
    resetPeakTracking();

    drive.stop();
}


void start()
{
    resetDetectionFilter();
    resetPeakTracking();

    changeState(
        AlignState::STRAFE_RIGHT
    );
}


// --------------------------------------------------
// Update
// --------------------------------------------------

void update()
{
    // UART::update() must still be called regularly
    // from main.cpp.
    updateDetectionFilter();


    switch (currentState)
    {
        // --------------------------------------------------

        case AlignState::IDLE:
        {
            break;
        }


        // --------------------------------------------------
        // Initial lateral positioning
        // --------------------------------------------------

        case AlignState::STRAFE_RIGHT:
        {
            if (
                millis() - stateStartTime >=
                STRAFE_TIME_MS
            )
            {
                changeState(
                    AlignState::SEARCH_BACKWARD
                );
            }

            break;
        }


        // --------------------------------------------------
        // Move backward to get to one side of the peak
        // --------------------------------------------------

        case AlignState::SEARCH_BACKWARD:
        {
            drive.backward(
                BACKWARD_SPEED
            );

            if (
                millis() - stateStartTime >=
                BACKWARD_TIME_MS
            )
            {
                changeState(
                    AlignState::SEARCH_FORWARD
                );
            }

            break;
        }


        // --------------------------------------------------
        // Precision forward sweep
        // --------------------------------------------------

        case AlignState::SEARCH_FORWARD:
        {
            drive.forward(
                FORWARD_SCAN_SPEED
            );


            // Track the maximum magnitude and wait until
            // we've clearly moved past it.
            if (updatePeakTracking())
            {
                changeState(
                    AlignState::RETURN_TO_PEAK
                );

                break;
            }


            // Failed to find a real peak.
            if (
                millis() - stateStartTime >=
                FORWARD_SEARCH_TIME_MS
            )
            {
                changeState(
                    AlignState::NOT_FOUND
                );
            }


            break;
        }


        // --------------------------------------------------
        // Reverse back toward the measured peak
        // --------------------------------------------------

        case AlignState::RETURN_TO_PEAK:
        {
            drive.backward(
                RETURN_SPEED
            );


            if (
                millis() - stateStartTime >=
                returnTimeMs
            )
            {
                changeState(
                    AlignState::FINISHED
                );
            }


            break;
        }


        // --------------------------------------------------

        case AlignState::FINISHED:
        case AlignState::NOT_FOUND:
        {
            drive.stop();
            break;
        }
    }
}


// --------------------------------------------------
// Stop
// --------------------------------------------------

void stop()
{
    changeState(
        AlignState::IDLE
    );
}


// --------------------------------------------------
// Status
// --------------------------------------------------

bool isFinished()
{
    return
        currentState ==
        AlignState::FINISHED;
}


bool hasFailed()
{
    return
        currentState ==
        AlignState::NOT_FOUND;
}


bool isDone()
{
    return
        isFinished() ||
        hasFailed();
}


// --------------------------------------------------
// Telemetry
// --------------------------------------------------

uint16_t getCurrentMagnitude()
{
    return getFilteredMagnitude();
}


uint16_t getMaximumMagnitude()
{
    return maximumMagnitude;
}


// --------------------------------------------------
// State name
// --------------------------------------------------

const char* getStateName()
{
    switch (currentState)
    {
        case AlignState::IDLE:
            return "Idle";

        case AlignState::STRAFE_RIGHT:
            return "Strafe Right";

        case AlignState::SEARCH_BACKWARD:
            return "Move to Start";

        case AlignState::SEARCH_FORWARD:
            return "Peak Scan";

        case AlignState::RETURN_TO_PEAK:
            return "Return to Peak";

        case AlignState::FINISHED:
            return "IR Aligned";

        case AlignState::NOT_FOUND:
            return "IR Peak Not Found";
    }

    return "Unknown";
}
*/

} // namespace IRAligner