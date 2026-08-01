#include "core/states/SlowTapeFollowing.h"

#include <Arduino.h>

#include "tape_logic/TapeFollower.h"

namespace SlowTapeFollowing
{

static constexpr int TAPE_SPEED = 60;

static constexpr uint16_t MAG1_THRESHOLD = 20000;
static constexpr uint16_t MAG2_THRESHOLD = 3000;

static constexpr int SENSOR_SELECT_PIN = 11;

static bool running = false;

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

    running = true;
}

void update()
{
    if (!running)
    {
        return;
    }

    tapeFollowStep();
}

void stop()
{
    setTapeFollowing(false);
    running = false;
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

} // namespace SlowTapeFollowing