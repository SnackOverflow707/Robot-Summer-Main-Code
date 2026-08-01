
#include "core/states/IRAlignerManual.h"

#include <Arduino.h>
#include <math.h>

#include "actuators/MecanumDrive.h"
#include "comms/UART.h"

extern MecanumDrive drive;

namespace IRAlignerManual
{

// --------------------------------------------------
// Configuration
// --------------------------------------------------

// Target solar-panel location relative to the side-tape reference.
//
// Replace these values with your measured coordinates.
static constexpr float SOLAR_PANEL_FROM_SIDE_TAPES_DX = 0.0f;
static constexpr float SOLAR_PANEL_FROM_SIDE_TAPES_DY = 0.0f;

// Speed supplied to MecanumDrive::driveTo().
static constexpr int TRAVEL_SPEED = 80;

// Maximum allowed final position error.
static constexpr float ARRIVAL_TOLERANCE_M = 0.05f;

// --------------------------------------------------
// Internal state
// --------------------------------------------------

enum class ManualAlignState
{
    IDLE,
    DRIVING,
    FINISHED,
    FAILED
};

static ManualAlignState currentState =
    ManualAlignState::IDLE;

// --------------------------------------------------
// Internal helpers
// --------------------------------------------------

static void driveToSolarPanelCoordinates(
    float currentX,
    float currentY
)
{
    const float travelX =
        SOLAR_PANEL_FROM_SIDE_TAPES_DX - currentX;

    const float travelY =
        SOLAR_PANEL_FROM_SIDE_TAPES_DY - currentY;

    drive.driveTo(
        travelX,
        travelY,
        TRAVEL_SPEED
    );
}

static float calculateRemainingDistance(
    float currentX,
    float currentY
)
{
    const float errorX =
        SOLAR_PANEL_FROM_SIDE_TAPES_DX - currentX;

    const float errorY =
        SOLAR_PANEL_FROM_SIDE_TAPES_DY - currentY;

    return sqrtf(
        errorX * errorX +
        errorY * errorY
    );
}

// --------------------------------------------------
// Public state controls
// --------------------------------------------------

void begin()
{
    drive.stop();

    currentState =
        ManualAlignState::IDLE;
}

void start()
{
    if (currentState == ManualAlignState::DRIVING)
    {
        return;
    }

    const UART::PoseData startPose =
        UART::getPoseData();

    if (!startPose.valid)
    {
        Serial.println(
            "[IRAlignerManual] Invalid starting pose"
        );

        drive.stop();

        currentState =
            ManualAlignState::FAILED;

        return;
    }

    currentState =
        ManualAlignState::DRIVING;

    Serial.printf(
        "[IRAlignerManual] Starting at x=%.3f y=%.3f\n",
        startPose.x,
        startPose.y
    );

    /*
     * MecanumDrive::driveTo() is assumed to block until it reaches
     * its target tolerance or times out.
     */
    driveToSolarPanelCoordinates(
        startPose.x,
        startPose.y
    );

    drive.stop();

    const UART::PoseData finalPose =
        UART::getPoseData();

    if (!finalPose.valid)
    {
        Serial.println(
            "[IRAlignerManual] Invalid final pose"
        );

        currentState =
            ManualAlignState::FAILED;

        return;
    }

    const float remainingDistance =
        calculateRemainingDistance(
            finalPose.x,
            finalPose.y
        );

    Serial.printf(
        "[IRAlignerManual] Final x=%.3f y=%.3f error=%.3f m\n",
        finalPose.x,
        finalPose.y,
        remainingDistance
    );

    if (remainingDistance <= ARRIVAL_TOLERANCE_M)
    {
        currentState =
            ManualAlignState::FINISHED;
    }
    else
    {
        currentState =
            ManualAlignState::FAILED;
    }
}

void update()
{
    /*
     * Nothing is needed here because start() currently calls the
     * blocking MecanumDrive::driveTo() function.
     *
     * If driveTo() becomes non-blocking later, its progress checks
     * should be moved here.
     */
}

void stop()
{
    drive.stop();

    currentState =
        ManualAlignState::IDLE;
}

// --------------------------------------------------
// Status
// --------------------------------------------------

bool isFinished()
{
    return currentState ==
        ManualAlignState::FINISHED;
}

bool hasFailed()
{
    return currentState ==
        ManualAlignState::FAILED;
}

bool isDone()
{
    return isFinished() || hasFailed();
}

const char* getStateName()
{
    switch (currentState)
    {
        case ManualAlignState::IDLE:
            return "Idle";

        case ManualAlignState::DRIVING:
            return "Driving to Solar Panel";

        case ManualAlignState::FINISHED:
            return "Manual Alignment Finished";

        case ManualAlignState::FAILED:
            return "Manual Alignment Failed";
    }

    return "Unknown";
}

} // namespace IRAlignerManual

