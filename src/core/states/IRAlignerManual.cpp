#include "core/states/IRAlignerManual.h"
#include "core/states/SlowTapeFollowing.h"

#include <Arduino.h>

#include "actuators/MecanumDrive.h"
#include "comms/UART.h"

extern MecanumDrive drive;

namespace IRAlignerManual
{

    // How close (in meters) we need to end up to the target coordinates
    // to call the manual alignment a success. 
    static constexpr float ARRIVAL_TOLERANCE_M = 0.05f;

    enum class ManualAlignState
    {
        IDLE,
        DRIVING,
        FINISHED,
        FAILED
    };

    static ManualAlignState currentState = ManualAlignState::IDLE;

    static void driveToSolarPanels(float currentX, float currentY) {

        float travelX = SOLAR_PANEL_FROM_SIDE_TAPES_DX - currentX;
        float travelY = SOLAR_PANEL_FROM_SIDE_TAPES_TOWER_DY - currentY;

        drive.driveTo(travelX, travelY, TRAVEL_SPEED);

    }


    void begin() {
        currentState = ManualAlignState::IDLE;
        drive.stop();
    }


    // Drives straight to the known solar-panel coordinates (measured
    // relative to the tower/side-tape reference frame) using dead
    // reckoning from the flow sensor, since IR search has already
    // failed. MecanumDrive::driveTo() runs its own loop internally
    // until it reaches tolerance or times out, so this call blocks
    // until the attempt is over -- there's no separate update() step.
    void driveToSolarPanel() {

        const UART::PoseData startPose = UART::getPoseData();

        if (!startPose.valid) {
            currentState = ManualAlignState::FAILED;
            return;
        }

        currentState = ManualAlignState::DRIVING;

        driveToSolarPanels(startPose.x, startPose.y);

        const UART::PoseData finalPose = UART::getPoseData();

        if (!finalPose.valid) {
            currentState = ManualAlignState::FAILED;
            return;
        }

        const float errorX = SOLAR_PANEL_FROM_SIDE_TAPES_DX - finalPose.x;
        const float errorY = SOLAR_PANEL_FROM_SIDE_TAPES_TOWER_DY - finalPose.y;
        const float remainingDistance = sqrtf(errorX * errorX + errorY * errorY);

        currentState = (remainingDistance <= ARRIVAL_TOLERANCE_M)
            ? ManualAlignState::FINISHED
            : ManualAlignState::FAILED;
    }


    void stop() {
        drive.stop();
        currentState = ManualAlignState::IDLE;
    }


    bool isFinished() {
        return currentState == ManualAlignState::FINISHED;
    }


    bool hasFailed() {
        return currentState == ManualAlignState::FAILED;
    }


    bool isDone() {
        return isFinished() || hasFailed();
    }


    const char* getStateName() {
        switch (currentState)
        {
            case ManualAlignState::IDLE:
                return "Idle";

            case ManualAlignState::DRIVING:
                return "Driving to Solar Panel (Manual)";

            case ManualAlignState::FINISHED:
                return "Manual Alignment Finished";

            case ManualAlignState::FAILED:
                return "Manual Alignment Failed";
        }

        return "Unknown";
    }

}; // namespace IRAlignerManual
