#include "core/states/RockApproach.h"

#include "actuators/MecanumDrive.h"
#include "tape_logic/TapeFollower.h"
#include "comms/UART.h"

extern MecanumDrive drive;

namespace RockApproach
{

        
const RockPos ROCK_POSITIONS[6] = {
    {  40.1208f,  -723.5691f, 1, true  },
    { -64.6298f, -1183.8174f, 0, true  },
    { 117.5088f, -1704.1790f, 1, false },
    { 462.3132f, -2360.1294f, 1, false },
    { 610.5424f, -4324.3765f, 0, false },
    { 327.8498f, -4615.5718f, 1, true  }
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
    #define STRAFE_AMOUNT 20.0f //UPDATE
    #define Y_TOLERANCE 10.0f //pixels on y (along tape)

    // States
    enum class Phase {
        AWAIT_Y,
        STRAFE_TO_SCAN,
        METAL_CHECK,
        DONE,
        FAILED
    };

    static Phase s_phase = Phase::DONE;
    static uint8_t s_rockIndex = 0;
    static float s_strafeStartX = 0.0f;

    static float getPoseX() { return UART::getPoseData().x; }
    static float getPoseY() { return UART::getPoseData().y; }

    // static float distance(float x1, float y1, float x2, float y2) {
    //     float dx = x2 - x1;
    //     float dy = y2 - y1;

    //     return sqrtf(dx*dx + dy*dy);
    // }

    void begin() {}

    void start(uint8_t rockIndex) {
        s_rockIndex = rockIndex;
        s_phase = Phase::AWAIT_Y;
        drive.stop();
    }

    void update() {
    //if (s_phase == Phase::DONE || s_phase == Phase::FAILED) return;
    UART::update();
    float px = getPoseX();
    float py = getPoseY();

    const ScanPos& scan = SCAN_POSITIONS[s_rockIndex];
    const RockPos& rp = ROCK_POSITIONS[s_rockIndex];

    switch (s_phase) {

        case Phase::AWAIT_Y: {
            // just wait until Y is close to rock 
            float dy = fabsf(py - rp.y);
                drive.stop();
                if (rp.strafe) {
                    s_strafeStartX = px;   // record X when we stop
                    s_phase = Phase::STRAFE_TO_SCAN;
                } else{
                    s_phase = Phase::METAL_CHECK; //no srafing required
                }
            break;
        }

        case Phase::STRAFE_TO_SCAN: {
            float strafed = fabsf(px - s_strafeStartX);

            if (strafed >= STRAFE_AMOUNT) {
                drive.stop();
                s_phase = Phase::METAL_CHECK;
                break;
            }

            // strafe direction based on coil side
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

        default: break;
    }
}

void stop() {
    drive.stop();
    s_phase = Phase::FAILED;
}

bool isFinished() { return s_phase == Phase::DONE; }
bool hasFailed()  { return s_phase == Phase::FAILED; }
bool isWaitingForY() { return s_phase == Phase::AWAIT_Y; }

const RockPos* getRockPositions() { return ROCK_POSITIONS; }

} // namespace RockApproach
