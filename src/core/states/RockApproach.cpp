#include "core/states/RockApproach.h"

#include "actuators/MecanumDrive.h"
#include "tape_logic/TapeFollower.h"
#include "comms/UART.h"
#include "core/StateMachine.h"
#include "core/states/CourseTimeBudget.h"

extern MecanumDrive drive;

namespace RockApproach
{

    constexpr RockPos ROCK_POSITIONS[6] = {
        //UPDATE WITH REAL POSES!!
        // 0 -> left coil
        // 1 -> right coil
        // (AS VIEWED FROM THE REAR)
        
        {96.0217f, -781.6851f, 1}, 
        {1000.0f, 2000.0f, 0},
        {1000.0f, 2000.0f, 0},
        {1000.0f, 2000.0f, 0},
        {1000.0f, 2000.0f, 0},
        {1000.0f, 2000.0f, 0}
    };

    struct ScanPos {
        float x;
        float y;
    };

    static constexpr ScanPos SCAN_POSITIONS[6] = {
        //UPDATE!
        {193.3023f, -770.2703f}, 
        {1000.0f, 2000.0f}, 
        {1000.0f, 2000.0f}, 
        {1000.0f, 2000.0f}, 
        {1000.0f, 2000.0f}, 
        {1000.0f, 2000.0f}, 
    };

    #define STRAFE_SPEED 150
    #define POSITION_TOLERANCE 80.0f //pixels

    // States
    enum class Phase {
        STRAFE_TO_SCAN,
        METAL_CHECK,
        STRAFE_TO_TAPE,
        DONE,
        FAILED
    };

    static Phase s_phase = Phase::DONE;
    static uint8_t s_rockIndex = 0;

    static float getPoseX() { return UART::getPoseData().x; }
    static float getPoseY() { return UART::getPoseData().y; }

    static float distance(float x1, float y1, float x2, float y2) {
        float dx = x2 - x1;
        float dy = y2 - y1;

        return sqrtf(dx*dx + dy*dy);
    }

    void begin() {}

    void start(uint8_t rockIndex) {
        // Go/no-go: don't commit to another rock if there isn't enough
        // course time left to attempt it and still make the tower/panels.
        if (StateMachine::getCourseElapsedMs() + ROCK_APPROACH_WORST_CASE_MS >
            ROCK_TIME_DEADLINE_MS) {
            s_phase = Phase::FAILED;
            return;
        }

        s_rockIndex = rockIndex;
        s_phase = Phase::STRAFE_TO_SCAN;
        drive.stop();
    }

    void update() {
    if (s_phase == Phase::DONE || s_phase == Phase::FAILED) return;

    float px = getPoseX();
    float py = getPoseY();

    const ScanPos& scan = SCAN_POSITIONS[s_rockIndex];
    const RockPos& rp = ROCK_POSITIONS[s_rockIndex];

    switch (s_phase) {

        case Phase::STRAFE_TO_SCAN: {
            float d = distance(px, py, scan.x, scan.y);

            // stops when POSITION_TOLERANCE pixels from rock
            if (d < POSITION_TOLERANCE) {
                drive.stop();
                s_phase = Phase::METAL_CHECK;
                break;
            }

            if (rp.coil == 0) 
                drive.strafeLeft(STRAFE_SPEED);
            else
                drive.strafeRight(STRAFE_SPEED);
            
            break;
        }

        case Phase::METAL_CHECK:
            // stop here — StateMachine transitions to ROCK_METAL_CHECK
            // then back to ROCK_APPROACH which calls update() again
            // at that point we move to return phase
            drive.stop();
            s_phase = Phase::DONE; //goes to ROCK_METAL_CHECK
            break;

        case Phase::STRAFE_TO_TAPE: {
            // strafe back toward tape until sensor detects it
            if (scan.y > ROCK_POSITIONS[s_rockIndex].y)
                drive.strafeRight(STRAFE_SPEED);
            else
                drive.strafeLeft(STRAFE_SPEED);

            TapeFollowerStatus status = getTapeFollowerStatus();
            if (status.leftWhite || status.rightWhite) {
                drive.stop();
                s_phase = Phase::DONE;
            }
            break;
        }

        default: break;
    }
}
void stop() {
    drive.stop();
    s_phase = Phase::FAILED;
}

bool isFinished() { return s_phase == Phase::DONE; }
bool hasFailed()  { return s_phase == Phase::FAILED; }

const RockPos* getRockPositions() { return ROCK_POSITIONS; }

} // namespace RockApproach
